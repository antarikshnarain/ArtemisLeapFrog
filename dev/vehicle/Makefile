setup-exec:
	chmod +x ~/vehicle/run_leapos.sh
	sudo cp ~/vehicle/run_leapos.sh /usr/local/bin/

setup-service:
	sudo cp ~/vehicle/vehicle.service /etc/systemd/system/vehicle.service
	sudo systemctl daemon-reload

install-service: setup-exec setup-service 
	sudo systemctl enable vehicle.service

start-service: setup-exec setup-service
	sudo systemctl start vehicle.service

stop-service: setup-exec setup-service
	sudo systemctl stop vehicle.service

uninstall-service: 
	sudo systemctl disable vehicle.service

build:
	colcon build

clean:
	rm -r build/ log/ install/