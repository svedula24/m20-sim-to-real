# Common entry points. `make help` lists targets.
.PHONY: help unit image nlnav-image test up down

help:
	@echo "unit        - fast ROS-free logic tests (pytest + shims, runs anywhere)"
	@echo "image       - build the stack image m20_autonomy:latest"
	@echo "nlnav-image - build m20_nlnav:latest (stack + torch + OWLv2) for NL navigation and the object map"
	@echo "test        - colcon test inside the stack image (integration)"
	@echo "up          - docker compose up the robot stack (commander, bridge, mission server, station)"
	@echo "down        - docker compose down"

unit:
	python3 -m pytest tests -q

image:
	docker build -f docker/Dockerfile -t m20_autonomy:latest .

nlnav-image: image
	docker build -f docker/Dockerfile.nlnav -t m20_nlnav:latest .

test: image
	docker run --rm m20_autonomy:latest bash -lc \
	  "source /ws/install/setup.bash && cd /ws && colcon test && colcon test-result --verbose"

up:
	docker compose -f docker/compose.yaml up -d

down:
	docker compose -f docker/compose.yaml down
