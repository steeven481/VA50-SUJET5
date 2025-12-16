#!/usr/bin/env python3
import subprocess
import time


# ----------------------------------------------------------------------
# Utilitaires affichage
# ----------------------------------------------------------------------
def print_step(title):
    print("\n" + "=" * 70)
    print(f" 🟦 {title}")
    print("=" * 70 + "\n")
    time.sleep(0.2)


def run(cmd):
    print(f"➡️  Commande exécutée : {' '.join(cmd)}")
    print("-" * 70)
    subprocess.run(cmd, check=True)
    print("-" * 70 + "\n")


# ----------------------------------------------------------------------
# Pipeline principal
# ----------------------------------------------------------------------
def main():

    print("\n" + "#" * 70)
    print(" 🚀 PIPELINE VA50 — EXÉCUTION COMPLÈTE ")
    print("#" * 70 + "\n")
    time.sleep(0.3)

    timings = {}
    total_start = time.time()

    # ----------------------------------------------------------
    # STEP 1 — Détection sur l'image réelle (positions initiales)
    # ----------------------------------------------------------
    print_step("STEP 1 — Détection objets (image réelle)")

    t0 = time.time()
    run([
        "python3", "src/detect_objects.py",
        "--image", "data/images/table_input.png",
        "--output", "outputs/detections_input.json"
    ])
    timings["Détection image réelle"] = time.time() - t0

    # ----------------------------------------------------------
    # STEP 2 — Génération de l'image cible (ControlNet)
    # ----------------------------------------------------------
    print_step("STEP 2 — Génération image cible (ControlNet)")

    t0 = time.time()
    run([
        "python3", "src/img-gene.py"
    ])
    timings["Génération image"] = time.time() - t0

    # ----------------------------------------------------------
    # STEP 3 — Extraction positions finales depuis image générée
    # ----------------------------------------------------------
    print_step("STEP 3 — Calcul positions finales (image générée)")

    t0 = time.time()
    run([
        "python3", "src/generated_image_to_positions.py",
        "--image", "outputs/img_generated.png",
        "--output", "outputs/final_positions.json",
        "--debug_image", "outputs/debug_detected_positions.png"
    ])
    timings["Positions finales (pixels)"] = time.time() - t0

    # ----------------------------------------------------------
    # FIN PIPELINE
    # ----------------------------------------------------------
    total_time = time.time() - total_start

    print("\n" + "#" * 70)
    print(" 🎉 PIPELINE VA50 TERMINÉ AVEC SUCCÈS !")
    print("#" * 70)

    print("\n⏱️  RÉCAPITULATIF DES TEMPS :")
    print("-" * 70)

    for step_name, duration in timings.items():
        print(f" 🔹 {step_name:<40} : {duration:6.2f} sec")

    print("-" * 70)
    print(f" 🟩 Temps total pipeline                 : {total_time:6.2f} sec")
    print("-" * 70)

    print("\n📌 Fichiers générés :")
    print("   - outputs/detections_input.json   (positions initiales)")
    print("   - outputs/img_generated.png       (image cible)")
    print("   - outputs/final_positions.json    (positions robot)")
    print("   - outputs/debug_detected_positions.png (debug visuel)\n")

    print(" ✔ Le robot peut maintenant utiliser final_positions.json\n")


if __name__ == "__main__":
    main()
