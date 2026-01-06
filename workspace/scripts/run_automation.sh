#!/bin/bash
# Script de lancement - Pipeline de réarrangement avec Tiago
#
# Usage:
#   ./run_automation.sh [full|capture|pipeline|command] [--no-wait]

# Couleurs
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    echo ""
    echo -e "${BLUE}==============================================================================${NC}"
    echo -e "${BLUE} $1${NC}"
    echo -e "${BLUE}==============================================================================${NC}"
    echo ""
}

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

print_header "Pipeline de Rearrangement Autonome avec Tiago"

if [ ! -f "$SCRIPT_DIR/automation_node.py" ]; then
    print_error "Script automation_node.py non trouvé dans $SCRIPT_DIR"
    exit 1
fi

if [ -f "/opt/pal/gallium/setup.bash" ]; then
    print_info "Sourçage de l'environnement PAL Robotics..."
    source /opt/pal/gallium/setup.bash
fi

if [ -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
    print_info "Sourçage du workspace ROS..."
    source "$WORKSPACE_DIR/devel/setup.bash"
fi

print_info "Workspace: $WORKSPACE_DIR"
print_info "Scripts: $SCRIPT_DIR"

MODE="full"
NO_WAIT=""

for arg in "$@"; do
    case $arg in
        capture|pipeline|command|full)
            MODE="$arg"
            ;;
        --no-wait)
            NO_WAIT="--no-wait"
            ;;
        --help|-h)
            echo "Usage: $0 [mode] [options]"
            echo ""
            echo "Modes:"
            echo "  full      - Pipeline complet (défaut)"
            echo "  capture   - Capture d'images uniquement"
            echo "  pipeline  - Pipeline IA uniquement"
            echo "  command   - Commande robot uniquement"
            echo ""
            echo "Options:"
            echo "  --no-wait - Ne pas attendre que Gazebo soit prêt"
            echo "  --help    - Afficher cette aide"
            exit 0
            ;;
        *)
            print_warning "Argument inconnu: $arg"
            ;;
    esac
done

print_info "Mode d'exécution: $MODE"

print_header "Demarrage du Pipeline"

cd "$SCRIPT_DIR"
python3 automation_node.py --mode "$MODE" $NO_WAIT

EXIT_CODE=$?

if [ $EXIT_CODE -eq 0 ]; then
    print_header "Pipeline Termine avec Succes"
else
    print_header "Pipeline Termine avec des Erreurs"
    print_error "Code de sortie: $EXIT_CODE"
fi

exit $EXIT_CODE
