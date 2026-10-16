"""장애물 윤곽선 -> L515 컬러(1280x720) 픽셀 좌표 -> UDP JSON 패킷.
open3d 불필요. terrain.py 의 결과(obst_mask, fwd, lat)만 받아서 동작."""
import json, time
import numpy as np
import cv2

COLOR_W, COLOR_H = 1280, 720
MAX_BYTES = 1200          # UDP 한 패킷 (단편화 방지)
MAX_OBST, MAX_PTS, MIN_AREA = 4, 16, 150


class ColorProjector:
    """깊이 픽셀 -> 컬러 픽셀. 공칭 외부파라미터(URDF): p_c = p_d - t, 회전 없음."""
    def __init__(self, depth_K, color_K, t=(0.0, -0.024, 0.0)):
        self.dfx, self.dfy, self.dcx, self.dcy = depth_K
        self.cfx, self.cfy, self.ccx, self.ccy = color_K
        self.t = np.asarray(t, float)

    def mask_to_color(self, mask, depth_m):
        v, u = np.nonzero(mask)
        z = depth_m[v, u]
        ok = z > 0
        u, v, z = u[ok], v[ok], z[ok]
        x = (u - self.dcx) * z / self.dfx - self.t[0]
        y = (v - self.dcy) * z / self.dfy - self.t[1]
        z = z - self.t[2]
        uc = np.round(x * self.cfx / z + self.ccx).astype(int)
        vc = np.round(y * self.cfy / z + self.ccy).astype(int)
        inside = (uc >= 0) & (uc < COLOR_W) & (vc >= 0) & (vc < COLOR_H)  # 컬러 화각 밖은 버림
        out = np.zeros((COLOR_H, COLOR_W), np.uint8)
        out[vc[inside], uc[inside]] = 255
        return out, u[ok][inside], v[ok][inside], z[inside]


def extract_polys(proj, obst_mask, depth_m):
    cm, _, _, _ = proj.mask_to_color(obst_mask, depth_m)
    # 깊이(640x480) -> 컬러(1280x720)는 점이 듬성하므로 메워준다
    cm = cv2.morphologyEx(cm, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cm = cv2.dilate(cm, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(cm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    polys = []
    for c in cnts:
        if cv2.contourArea(c) < MIN_AREA:
            continue
        eps = 0.01 * cv2.arcLength(c, True)
        ap = cv2.approxPolyDP(c, eps, True)
        while len(ap) > MAX_PTS:
            eps *= 1.5
            ap = cv2.approxPolyDP(c, eps, True)
        polys.append(ap.reshape(-1, 2).tolist())
    return polys


def build_packet(proj, seq, status, obst_mask, depth_m, fwd, lat, zones,
                 nearest, half_width, cam_h=None, tilt=None, proc_ms=None):
    """status != OK 이면 obstacles 는 비우고 status 로 '판단 불가'를 알린다."""
    pkt = {"schema_version": 1, "type": "terrain", "sequence": seq,
           "stamp": round(time.time(), 3), "status": status,
           "frame_size": [COLOR_W, COLOR_H],
           "nearest_obstacle_m": None if nearest is None else round(float(nearest), 2),
           "zones": zones, "obstacles": [],
           "camera_height_m": cam_h, "camera_tilt_deg": tilt, "proc_ms": proc_ms}
    if status == "OK" and obst_mask is not None and obst_mask.any():
        polys = extract_polys(proj, obst_mask, depth_m)
        pkt["obstacles"] = _attach_dist(proj, polys, obst_mask, depth_m, fwd, lat, half_width)
    s = json.dumps(pkt, separators=(",", ":"))
    while len(s.encode()) > MAX_BYTES and pkt["obstacles"]:
        if any(len(o["poly"]) > 6 for o in pkt["obstacles"]):
            for o in pkt["obstacles"]:
                if len(o["poly"]) > 6:
                    o["poly"] = o["poly"][::2]
        else:
            pkt["obstacles"].pop()
        s = json.dumps(pkt, separators=(",", ":"))
    return pkt, s.encode()


def _attach_dist(proj, obs_polys, obst_mask, depth_m, fwd, lat, half_width):
    v, u = np.nonzero(obst_mask)
    z = depth_m[v, u]; ok = z > 0
    u, v, z = u[ok], v[ok], z[ok]
    x = (u - proj.dcx) * z / proj.dfx - proj.t[0]
    y = (v - proj.dcy) * z / proj.dfy - proj.t[1]
    zz = z - proj.t[2]
    uc = np.round(x * proj.cfx / zz + proj.ccx)
    vc = np.round(y * proj.cfy / zz + proj.ccy)
    f = fwd[v, u]; l = lat[v, u]
    res = []
    for poly in obs_polys:
        p = np.array(poly, np.int32).reshape(-1, 1, 2)
        m = np.zeros((COLOR_H, COLOR_W), np.uint8)
        cv2.fillPoly(m, [p], 255)
        ui = np.clip(uc.astype(int), 0, COLOR_W - 1); vi = np.clip(vc.astype(int), 0, COLOR_H - 1)
        inside = (m[vi, ui] > 0) & (uc >= 0) & (uc < COLOR_W) & (vc >= 0) & (vc < COLOR_H)
        if not inside.any():
            continue
        fi, li = f[inside], l[inside]
        res.append({"dist_m": round(float(np.percentile(fi, 5)), 2),
                    "in_path": bool((np.abs(li) < half_width).any()),
                    "poly": poly})
    res.sort(key=lambda o: o["dist_m"])
    return res[:MAX_OBST]
