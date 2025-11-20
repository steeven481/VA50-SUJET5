import argparse
import os
import cv2
import numpy as np
import torch
import torchvision
from torchvision.transforms.functional import to_tensor

from utils import write_json, pixels_to_table

# ======================================================
# 0. Noms COCO 
# ======================================================
COCO_INSTANCE_CATEGORY_NAMES = [
    '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane',
    'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
    'street sign', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
    'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
    'hat', 'backpack', 'umbrella', 'shoe', 'eye glasses', 'handbag', 'tie',
    'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite',
    'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
    'tennis racket', 'bottle', 'plate', 'wine glass', 'cup', 'fork',
    'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
    'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
    'couch', 'potted plant', 'bed', 'mirror', 'dining table', 'window',
    'desk', 'toilet', 'door', 'tv', 'laptop', 'mouse', 'remote', 'keyboard',
    'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator',
    'blender', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush', 'hair brush', 'banner', 'blanket', 'branch',
    'bridge', 'building', 'bush', 'cabinet', 'cage', 'camera', 'can',
    'candle', 'cannon', 'canoe', 'carpet', 'cart', 'cat', 'chain', 'chair',
    'chessboard', 'clock', 'clouds', 'coat', 'coffin', 'coin', 'comet',
    'computer', 'cone', 'container', 'counter', 'crate', 'crosswalk sign',
    'cupboard', 'curtain', 'cushion', 'dam', 'deer', 'desk', 'dirt track',
    'door', 'drawer', 'dress', 'dune', 'ear', 'engine', 'eye', 'face',
    'fence', 'field', 'file cabinet', 'floor', 'flower', 'fog',
    'food', 'footpath', 'forest', 'forklift', 'fountain', 'fox',
    'furniture']



# ======================================================
# 1. Chargement du modèle COCO
# ======================================================
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"[detect_objects] Chargement de Mask R-CNN COCO sur {device}...")
model = torchvision.models.detection.maskrcnn_resnet50_fpn_v2(weights="DEFAULT")
model = model.to(device).eval()
print("[detect_objects] Modèle prêt.")


# ======================================================
# 2. Pré-traitement & visu
# ======================================================
def preprocess(img_bgr):
    return cv2.convertScaleAbs(img_bgr, alpha=1.6, beta=30)


def visualize_detections(img_bgr, dets, out_path):
    vis = img_bgr.copy()
    for d in dets:
        x = int(d["x_pixel"])
        y = int(d["y_pixel"])
        label = d.get("label", "obj")

        cv2.circle(vis, (x, y), 6, (0, 255, 0), thickness=-1)
        cv2.putText(vis, label, (x + 10, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    cv2.imwrite(out_path, vis)
    print(f"[detect_objects] Image annotée sauvegardée dans {out_path}")


# ======================================================
# 3. Inference Mask R-CNN
# ======================================================
def run_mask_rcnn(img_bgr, debug=False):
    img_bgr = preprocess(img_bgr)
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    tensor = to_tensor(img_rgb).to(device)

    with torch.no_grad():
        out = model([tensor])[0]

    scores = out["scores"].cpu().numpy()
    masks = out["masks"].cpu().numpy()
    labels = out["labels"].cpu().numpy()

    if len(scores) == 0:
        return [], []

    idx_sorted = np.argsort(-scores)
    idx_top3 = idx_sorted[:3]

    dets = []
    extracted_masks = []

    for i in idx_top3:
        mask = (masks[i, 0] > 0.5).astype(np.uint8)
        ys, xs = np.nonzero(mask)

        if len(xs) == 0:
            continue

        u = float(xs.mean())
        v = float(ys.mean())
        x_table, y_table = pixels_to_table(u, v)

        class_id = int(labels[i])
        coco_label = COCO_INSTANCE_CATEGORY_NAMES[class_id]
        print(f"[debug] Detected COCO label: {coco_label}")

        dets.append({
            "raw_index": int(i),
            "score": float(scores[i]),
            "coco_label": coco_label,
            "pixel_centroid": [u, v],
            "x": x_table,
            "y": y_table,
            "x_pixel": u,
            "y_pixel": v,
            "theta": 0.0
        })

        extracted_masks.append(mask)

    return dets, extracted_masks


# ======================================================
# 4. Attribution des labels (robuste, basé sur COCO)
# ======================================================
def assign_labels_from_coco(dets, masks):
    """
    Mappe directement les labels COCO vers :
      - fork
      - knife
      - plate  (via 'bowl')
    """

    mapping = {
        "frisbee": "plate",
        "toothbrush": "fork",
        "baseball bat": "knife"
    }


    dets_out = []
    for i, d in enumerate(dets):
        d2 = d.copy()
        coco_label = d["coco_label"]

        d2["label"] = mapping.get(coco_label, "unknown")
        d2["id"] = f"obj_{i:03d}"

        dets_out.append(d2)

    return dets_out, masks


# ======================================================
# 5. Main
# ======================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--output", default="outputs/detections.json")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    img = cv2.imread(args.image)
    assert img is not None, f"Image introuvable : {args.image}"

    raw_dets, raw_masks = run_mask_rcnn(img, debug=args.debug)
    dets, masks = assign_labels_from_coco(raw_dets, raw_masks)

    write_json(dets, args.output)
    print(f"[detect_objects] {len(dets)} objets écrits dans {args.output}")

    out_dir = os.path.dirname(args.output)
    os.makedirs(out_dir, exist_ok=True)

    # Sauvegarde des masques individuels
    if masks:
        for d, mask in zip(dets, masks):
            filename = os.path.join(out_dir, f"mask_{d['label']}.png")
            cv2.imwrite(filename, mask * 255)
            print(f"[detect_objects] Masque sauvegardé : {filename}")

        mask_all = np.zeros_like(masks[0], dtype=np.uint8)
        for m in masks:
            mask_all |= m

        cv2.imwrite(os.path.join(out_dir, "mask_all.png"), mask_all * 255)
    else:
        print("[detect_objects] Aucun masque dispo.")

    vis_path = args.output.replace(".json", "_vis.png")
    visualize_detections(img, dets, vis_path)


if __name__ == "__main__":
    main()
