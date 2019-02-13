#!/bin/bash

source jet_functions.sh

function help () {
cat <<-END
Usage: jet build

Builds a new jet docker image for your host's architecture.

-h| --help           Show this message
pull| --pull         Pull the most recent version of the hoverjet/jet docker image
-p| --push           Push this image to dockerhub on build completion
--latest             Tag the image as "latest" on build completion
--no-cache           Run a clean build of the image, reperforming every cached step
END
}

PUSH=0
LATEST=0

while test $# -gt 0; do
        case "$1" in
                -h|--help)
                        help
                        exit 0
                        ;;
                --pull|pull*)
                        PULL=1
                        shift
                        ;;
                -p|--push*)
                        PUSH=1
                        shift
                        ;;
                --latest*)
                        LATEST=1
                        shift
                        ;;
                --no-cache*)
                        NO_CACHE="--no-cache"
                        shift
                        ;;
                *)
                        break
                        ;;
        esac
done

JET_REPO_PATH=$(git rev-parse --show-toplevel)

if [ $? -ne 0 ]; then
    exit $?
fi

FULL_IMAGE_NAME=$(get_image_name)

if [[ $PULL -ne 0 ]]; then
    echo "Pulling $FULL_IMAGE_NAME:latest"
    docker pull $FULL_IMAGE_NAME:latest
    exit 0
fi

DATE=`date +%Y.%m.%d-%H.%M.%S`

FULL_IMAGE_NAME_TAG="$FULL_IMAGE_NAME:$DATE"

echo Building image: $FULL_IMAGE_NAME_TAG
docker build $NO_CACHE $JET_REPO_PATH/infrastructure/scripts/docker/ -t $FULL_IMAGE_NAME_TAG

if [[ $LATEST -ne 0 ]]; then
	LATEST_IMAGE_NAME_TAG="hoverjet/$IMAGE_NAME:latest"
	echo "Tagging $FULL_IMAGE_NAME_TAG as latest."
	docker tag $FULL_IMAGE_NAME_TAG $LATEST_IMAGE_NAME_TAG
fi

if [[ $PUSH -ne 0 ]]; then
	echo "Pushing $FULL_IMAGE_NAME_TAG"
	docker push $FULL_IMAGE_NAME_TAG
	if [[ $LATEST -ne 0 ]]; then
		echo "Pushing $LATEST_IMAGE_NAME_TAG"
		docker push $LATEST_IMAGE_NAME_TAG
	fi
fi
