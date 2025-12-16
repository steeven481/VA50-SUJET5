"""
Étape 2 : Caption (BLIP) + Vérification (CLIP) pour VA50-PIPELINE-DEV.

Objectif :
- Lire les objets détectés dans outputs/detections.json
- Découper les ROIs autour des centroïdes
- Générer une caption pour chaque ROI avec BLIP
- Vérifier la cohérence image ↔ texte avec CLIP
- Filtrer les objets incohérents
- Produire outputs/verified.json

Optimisations :
- BLIP + CLIP chargés une seule fois
- Traitement batch (plus rapide)
- Modes : both / blip_only / clip_only
"""

import argparse
import cv2
import torch
from transformers import (
    BlipProcessor,
    BlipForConditionalGeneration,
    CLIPProcessor,
    CLIPModel,
)

from utils import read_json, write_json

# =========================
# 1. Chargement des modèles
# =========================
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"[caption_and_clip] Chargement des modèles sur {device}...")

# BLIP pour captioning
blip_name = "Salesforce/blip-image-captioning-base"
blip_processor = BlipProcessor.from_pretrained(blip_name)
blip_model = BlipForConditionalGeneration.from_pretrained(blip_name).to(device).eval()

# CLIP pour vérification image <-> texte
clip_name = "openai/clip-vit-base-patch32"
clip_processor = CLIPProcessor.from_pretrained(clip_name)
clip_model = CLIPModel.from_pretrained(clip_name).to(device).eval()

print("[caption_and_clip] BLIP + CLIP prêts.")


# =========================
# 2. Utilitaires
# =========================
def crop_from_centroid(img_bgr, u: float, v: float, box: int = 160):
    """Découpe un carré centré sur (u,v) de taille `box`."""
    h, w = img_bgr.shape[:2]
    u0 = max(int(u - box // 2), 0)
    v0 = max(int(v - box // 2), 0)
    u1 = min(u0 + box, w)
    v1 = min(v0 + box, h)
    return img_bgr[v0:v1, u0:u1].copy()


def default_caption_from_label(label: str) -> str:
    """Caption fallback si on utilise CLIP seul."""
    base = {
        "fork": "a fork on a dark table",
        "knife": "a knife on a dark table",
        "plate": "a plate on a dark table",
    }
    return base.get(label, f"an object: {label}")


# =========================
# 3. BLIP en batch
# =========================
def blip_batch_captions(crops_bgr):
    if len(crops_bgr) == 0:
        return []

    crops_rgb = [cv2.cvtColor(r, cv2.COLOR_BGR2RGB) for r in crops_bgr]
    inputs = blip_processor(images=crops_rgb, return_tensors="pt", padding=True).to(device)

    with torch.no_grad():
        out = blip_model.generate(**inputs, max_new_tokens=20)

    return [
        blip_processor.decode(seq, skip_special_tokens=True).strip()
        for seq in out
    ]


# =========================
# 4. CLIP en batch
# =========================
def clip_batch_scores(crops_bgr, texts):
    if len(crops_bgr) == 0:
        return []

    crops_rgb = [cv2.cvtColor(r, cv2.COLOR_BGR2RGB) for r in crops_bgr]

    inputs = clip_processor(text=texts, images=crops_rgb,
                            return_tensors="pt", padding=True).to(device)

    with torch.no_grad():
        outputs = clip_model(**inputs)
        img_emb = outputs.image_embeds
        txt_emb = outputs.text_embeds

    # Normalisation L2
    img_emb = img_emb / img_emb.norm(p=2, dim=-1, keepdim=True)
    txt_emb = txt_emb / txt_emb.norm(p=2, dim=-1, keepdim=True)

    sims = (img_emb * txt_emb).sum(dim=-1)
    return [float(s.item()) for s in sims]


# =========================
# 5. Main
# =========================
def main():
    parser = argparse.ArgumentParser()

    # *** VALEURS PAR DÉFAUT POUR LANCER SANS PARAMÈTRES ***
    parser.add_argument("--input", default="outputs/detections.json",
                        help="Fichier JSON des détections.")
    parser.add_argument("--image", default="data/images/table_input.png",
                        help="Image source utilisée pour découper les ROIs.")
    parser.add_argument("--output", default="outputs/verified.json",
                        help="Fichier JSON filtré.")
    parser.add_argument("--roi", type=int, default=160,
                        help="Taille du ROI autour du centroïde.")
    parser.add_argument("--clip_min", type=float, default=0.28,
                        help="Score CLIP minimal pour valider un objet.")
    parser.add_argument("--mode",
                        choices=["both", "blip_only", "clip_only"],
                        default="both",
                        help="Mode : BLIP+CLIP, BLIP seul, CLIP seul.")

    args = parser.parse_args()

    # 1) Charger les entrées
    img = cv2.imread(args.image)
    assert img is not None, f"Image introuvable : {args.image}"

    dets = read_json(args.input)
    print(f"[caption_and_clip] {len(dets)} objets détectés en entrée.")

    if len(dets) == 0:
        write_json([], args.output)
        print("[caption_and_clip] Aucun objet → sortie vide.")
        return

    # 2) ROIs
    crops = [crop_from_centroid(img, *d["pixel_centroid"], box=args.roi)
             for d in dets]

    # 3) Génération des captions
    if args.mode in ("both", "blip_only"):
        captions = blip_batch_captions(crops)
    else:
        captions = [default_caption_from_label(d["label"]) for d in dets]

    # 4) Scores CLIP
    if args.mode in ("both", "clip_only"):
        scores = clip_batch_scores(crops, captions)
    else:
        scores = [1.0] * len(dets)

    # 5) Filtrage
    verified = []
    for d, cap, score in zip(dets, captions, scores):
        print(f" - {d['label']} : '{cap}' (CLIP={score:.3f})")

        if score >= args.clip_min:
            d["caption"] = cap
            d["clip_score"] = score
            verified.append(d)

    # 6) Sauvegarde
    write_json(verified, args.output)
    print(f"[caption_and_clip] gardé {len(verified)}/{len(dets)} objets → {args.output}")


if __name__ == "__main__":
    main()
