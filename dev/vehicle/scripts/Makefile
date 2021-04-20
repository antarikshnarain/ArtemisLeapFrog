generate-scripts:
	echo "Generating Scripts"
	python3 generator.py

install: generate-scripts
	echo "Installing Scripts"
	sudo mkdir -p /usr/local/share/script/
	sudo cp *.script /usr/local/share/script/
	ls /usr/local/share/script/ > custom_scripts.list
	sudo cp custom_scripts.list /usr/local/share/script/

uninstall: clean
	sudo rm -f /usr/local/share/script/*.script
clean:
	rm -f *.script custom_scripts.list