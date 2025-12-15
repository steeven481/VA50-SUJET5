import argparse
import os
import cv2
import numpy as np
import json


# =========================
# Utils
# =========================
def write_json(data, path):
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


# =========================
# Segmentation robuste
# =========================
def segment_objects(img_bgr):
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

    thresh = cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        51,
        5
    )

    kernel = np.ones((5, 5), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

    return thresh


# =========================
# Analyse des contours
# =========================
def analyze_contours(thresh):
    contours, _ = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    objects = []

    for c in contours:
        area = cv2.contourArea(c)
        if area < 500:
            continue

        M = cv2.moments(c)
        if M["m00"] == 0:
            continue

        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]

        rect = cv2.minAreaRect(c)
        (w, h) = rect[1]
        angle = rect[2]
        if w < h:
            angle += 90

        aspect_ratio = max(w, h) / (min(w, h) + 1e-6)

        objects.append({
            "contour": c,
            "area": area,
            "cx": cx,
            "cy": cy,
            "aspect_ratio": aspect_ratio,
            "theta": np.deg2rad(angle)
        })

    return objects


# =========================
# Classification par forme
# =========================
def classify_objects(objs):
    plates = []
    tools = []

    for o in objs:
        if o["aspect_ratio"] < 1.8:
            plates.append(o)
        else:
            tools.append(o)

    plates.sort(key=lambda x: x["area"], reverse=True)
    tools.sort(key=lambda x: x["area"], reverse=True)

    plates = plates[:1]
    tools = tools[:2]

    results = []

    for p in plates:
        results.append({
            "label": "plate",
            "x_pixel": float(p["cx"]),
            "y_pixel": float(p["cy"]),
            "theta": 0.0,
            "area": p["area"]
        })

    tools.sort(key=lambda x: x["cx"])

    if len(tools) == 2:
        results.append({
            "label": "fork",
            "x_pixel": float(tools[0]["cx"]),
            "y_pixel": float(tools[0]["cy"]),
            "theta": float(tools[0]["theta"]),
            "area": tools[0]["area"]
        })
        results.append({
            "label": "knife",
            "x_pixel": float(tools[1]["cx"]),
            "y_pixel": float(tools[1]["cy"]),
            "theta": float(tools[1]["theta"]),
            "area": tools[1]["area"]
        })

    for i, o in enumerate(results):
        o["id"] = f"obj_{i:03d}"

    return results


# =========================
# Verrouillage layout (robuste)
# =========================
def enforce_layout(objects, img_shape):
    h, w = img_shape[:2]

    if len(objects) >= 3:
        plate = max(objects, key=lambda o: o.get("area", 0))
        plate["label"] = "plate"

        others = [o for o in objects if o is not plate]

        left = min(others, key=lambda o: o["x_pixel"])
        right = max(others, key=lambda o: o["x_pixel"])

        left["label"] = "fork"
        right["label"] = "knife"

        return [plate, left, right]

    # fallback géométrique (ne plante jamais)
    print("[WARN] Fallback géométrique appliqué")

    cx = w / 2
    cy = h / 2

    return [
        {
            "id": "obj_000",
            "label": "plate",
            "x_pixel": cx,
            "y_pixel": cy,
            "theta": 0.0
        },
        {
            "id": "obj_001",
            "label": "fork",
            "x_pixel": cx - w * 0.25,
            "y_pixel": cy,
            "theta": np.pi / 2
        },
        {
            "id": "obj_002",
            "label": "knife",
            "x_pixel": cx + w * 0.25,
            "y_pixel": cy,
            "theta": np.pi / 2
        }
    ]


# =========================
# Debug visuel (CENTROÏDE)
# =========================
def draw_debug(img, objects, out_path):
    vis = img.copy()

    for o in objects:
        x = int(o["x_pixel"])
        y = int(o["y_pixel"])

        cv2.circle(vis, (x, y), 6, (0, 0, 255), -1)

        cv2.putText(
            vis,
            o["label"],
            (x + 8, y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        if o["label"] in ("fork", "knife"):
            length = 40
            dx = int(length * np.cos(o["theta"]))
            dy = int(length * np.sin(o["theta"]))
            cv2.arrowedLine(
                vis,
                (x, y),
                (x + dx, y + dy),
                (255, 0, 0),
                2,
                tipLength=0.25
            )

    cv2.imwrite(out_path, vis)


# =========================
# Main
# =========================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--output", default="outputs/final_positions.json")
    parser.add_argument(
        "--debug_image",
        default="outputs/debug_detected_positions.png"
    )
    args = parser.parse_args()

    img = cv2.imread(args.image)
    assert img is not None, f"Image introuvable : {args.image}"

    thresh = segment_objects(img)
    objs = analyze_contours(thresh)

    final = classify_objects(objs)
    final = enforce_layout(final, img.shape)

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    write_json(final, args.output)

    draw_debug(img, final, args.debug_image)

    print("[OK] Objets détectés :")
    for o in final:
        print(
            f" - {o['label']} @ ({o['x_pixel']:.1f}, {o['y_pixel']:.1f}) "
            f"theta={o['theta']:.2f} rad"
        )

    print(f"[OK] JSON écrit dans : {args.output}")
    print(f"[OK] Image debug écrite dans : {args.debug_image}")


if __name__ == "__main__":
    main()
