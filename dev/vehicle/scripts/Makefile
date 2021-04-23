generate-scripts:
	echo "Generating Scripts"
	python3 generator.py

uninstall: clean
	sudo rm -f /usr/local/share/script/*.script /usr/local/share/script/custom_scripts.list

install: uninstall generate-scripts 
	echo "Installing Scripts"
	sudo mkdir -p /usr/local/share/script/
	sudo cp *.script /usr/local/share/script/
	ls /usr/local/share/script/ > custom_scripts.list
	sudo cp custom_scripts.list /usr/local/share/script/

clean:
	rm -f *.script custom_scripts.list