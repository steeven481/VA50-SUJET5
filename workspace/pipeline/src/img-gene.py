import os
import time
import torch
from pathlib import Path
from PIL import Image



from diffusers import (
    StableDiffusionControlNetPipeline,
    ControlNetModel,
    DDIMScheduler
)

# ================================================================================
# 1) Chargement pipeline ONLINE (HF cache)
# ================================================================================
def load_generation_pipeline():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    dtype = torch.float16 if device == "cuda" else torch.float32

    print(f"[GEN] Device utilisé : {device}")

    # ControlNet Scribble
    controlnet = ControlNetModel.from_pretrained(
        "lllyasviel/sd-controlnet-scribble",
        torch_dtype=dtype
    )

    # Modèle PLUS obéissant pour géométrie
    pipe = StableDiffusionControlNetPipeline.from_pretrained(
        "lykon/dreamshaper-8",
        controlnet=controlnet,
        torch_dtype=dtype,
        safety_checker=None
    )

    # Scheduler stable
    pipe.scheduler = DDIMScheduler.from_config(pipe.scheduler.config)

    return pipe.to(device)


# ================================================================================
# 2) MAIN
# ================================================================================
def main():
    print("\n===============================")
    print("   🔵 IMG-GENE : CONTROLNET STRICT   ")
    print("===============================\n")

    t0 = time.time()
    pipe = load_generation_pipeline()

    # --- chemins robustes ---
    ROOT = Path(__file__).resolve().parent.parent
    scribble_path = ROOT / "data" / "scribble.png"

    if not scribble_path.exists():
        raise FileNotFoundError(f"Scribble introuvable : {scribble_path}")

    scribble = Image.open(scribble_path).convert("RGB")

    # --- PROMPT STRICT ---
    prompt = (
        "Top-down orthographic view. "
        "Flat neutral table surface. "
        "Exactly one round white plate centered in the image. "
        "Exactly one fork on the left side of the plate, horizontal, handle pointing down. "
        "Exactly one knife on the right side of the plate, horizontal, handle pointing down. "
        "No decorations. No colors. No design. "
        "Perfect symmetry. Simple geometry. Real-world scale."
    )

    negative_prompt = (
        "food, hands, people, colors, decoration, placemat, tray, "
        "square plate, rectangular plate,"
        "tilted objects, diagonal objects,"
        "artistic lighting, dramatic shadows, reflections,"
        "3d render, illustration, cartoon, painting"
    )

    print("[GEN] Génération image…")

    image = pipe(
        prompt=prompt,
        negative_prompt=negative_prompt,
        image=scribble,
        num_inference_steps=40,
        guidance_scale=5.5,
        controlnet_conditioning_scale=1.6,
        height=384,
        width=384
    ).images[0]

    # --- sauvegarde ---
    out_dir = ROOT / "outputs"
    out_dir.mkdir(exist_ok=True)

    out_path = out_dir / "img_generated.png"
    image.save(out_path)

    print(f"[GEN] Image générée → {out_path}")
    print(f"[GEN] Temps total : {time.time() - t0:.2f} sec")
    print("\n[IMG-GENE] ✔ Terminé.\n")


    
if __name__ == "__main__":
    main()
