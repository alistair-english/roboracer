#!/usr/bin/env bash
# Tag the local :dev image as the public submission tag and push to Docker Hub.
# Prompts for confirmation since this is visible to competition organizers.
set -euo pipefail

DEV_TAG="roboracer-icra2026:dev"
HUB_TAG="nineyards/roboracer-icra2026:practice"

if ! docker image inspect "${DEV_TAG}" >/dev/null 2>&1; then
  echo "ERROR: ${DEV_TAG} not found locally. Run 'pixi run docker_build' first."
  exit 1
fi

DEV_DIGEST=$(docker image inspect "${DEV_TAG}" --format '{{.Id}}')

echo "About to push the SUBMISSION image to Docker Hub."
echo
echo "  Local source:  ${DEV_TAG} (${DEV_DIGEST})"
echo "  Push target:   ${HUB_TAG}"
echo "  This is the image competition organizers will pull and evaluate."
echo
read -r -p "Are you sure? [y/N] " response
case "${response,,}" in
  y|yes) ;;
  *)
    echo "Aborted."
    exit 1
    ;;
esac

docker tag "${DEV_TAG}" "${HUB_TAG}"
docker push "${HUB_TAG}"

echo
echo "Pushed ${HUB_TAG}."
