import argparse
import os
import cv2
import numpy as np
import torch
import torchvision
from torchvision.transforms.functional import to_tensor
from utils import write_json


# =========================
# 0. Labels COCO
# =========================
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
    'hair drier', 'toothbrush', 'hair brush'
]


# =========================
# 1. Charger Mask R-CNN
# =========================
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"[detect_objects] Chargement de Mask R-CNN COCO sur {device}...")

model = torchvision.models.detection.maskrcnn_resnet50_fpn_v2(weights="DEFAULT")
model = model.to(device).eval()

print("[detect_objects] Modèle prêt.")


# =========================
# 2. Prétraitement
# =========================
def preprocess(img_bgr):
    return cv2.convertScaleAbs(img_bgr, alpha=1.6, beta=30)


# =========================
# 3. Inference + masques
# =========================
def run_mask_rcnn(img_bgr, masks_dir):
    img_bgr = preprocess(img_bgr)
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    h, w = img_bgr.shape[:2]

    tensor = to_tensor(img_rgb).to(device)

    with torch.no_grad():
        out = model([tensor])[0]

    scores = out["scores"].cpu().numpy()
    labels = out["labels"].cpu().numpy()
    masks = out["masks"].cpu().numpy()  # [N,1,H,W]

    if len(scores) == 0:
        return []

    os.makedirs(masks_dir, exist_ok=True)

    # masque global
    mask_all = np.zeros((h, w), dtype=np.uint8)

    # top-3 objets
    idx = np.argsort(-scores)[:3]
    results = []

    for i, obj_index in enumerate(idx):
        if scores[obj_index] < 0.15:
            continue

        mask = (masks[obj_index, 0] > 0.5).astype(np.uint8) * 255

        ys, xs = np.nonzero(mask)
        if len(xs) == 0:
            continue

        # centroïde
        u = float(xs.mean())
        v = float(ys.mean())

        coco_label = COCO_INSTANCE_CATEGORY_NAMES[int(labels[obj_index])]

        mapping = {
            "frisbee": "plate",
            "plate": "plate",
            "bowl": "plate",
            "fork": "fork",
            "toothbrush": "fork",
            "knife": "knife",
            "baseball bat": "knife"
        }

        final_label = mapping.get(coco_label, "unknown")

        obj_id = f"obj_{i:03d}"

        # sauvegarde masque individuel
        mask_path = os.path.join(
            masks_dir, f"mask_{obj_id}_{final_label}.png"
        )
        cv2.imwrite(mask_path, mask)

        # fusion dans mask_all
        mask_all = np.maximum(mask_all, mask)

        results.append({
            "id": obj_id,
            "label": final_label,
            "x_pixel": u,
            "y_pixel": v,
            "theta": 0.0,
            "mask_path": mask_path
        })

    # sauvegarde masque global
    cv2.imwrite(os.path.join(masks_dir, "mask_all.png"), mask_all)

    return results


# =========================
# 4. Main
# =========================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--output", default="outputs/detections.json")
    args = parser.parse_args()

    img = cv2.imread(args.image)
    assert img is not None, f"Image introuvable : {args.image}"

    masks_dir = os.path.join(os.path.dirname(args.output), "masks")

    detections = run_mask_rcnn(img, masks_dir)

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    write_json(detections, args.output)

    print(f"[detect_objects] Détections écrites dans : {args.output}")
    print(f"[detect_objects] Masques écrits dans : {masks_dir}")
    print(detections)


if __name__ == "__main__":
    main()
