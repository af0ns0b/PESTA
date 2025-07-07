import xml.etree.ElementTree as ET

def parse_kml(file_path):
	tree = ET.parse(file_path)
	root = tree.getroot()

    	
	ns = {"kml": "http://www.opengis.net/kml/2.2"}
	placemarks = root.findall(".//kml:Placemark", ns)

	waypoints = set() 
	for placemark in placemarks:
		coordinates = placemark.find(".//kml:coordinates", ns)
		if coordinates is not None:
			coord_sets = coordinates.text.strip().split()
			for coord_set in coord_sets:
				coord_list = coord_set.split(",")
				if len(coord_list) >= 3:  
					lon, lat, alt = coord_list[:3] 
					waypoints.add(f"{lon}, {lat}, {alt}")

	return list(waypoints)

def save_to_txt(waypoints, output_file):
    	with open(output_file, 'w') as f:
        	for waypoint in waypoints:
            		f.write(waypoint + '\n')

if __name__ == "__main__":
    	kml_file = "/home/afonso/catkin_ws/src/waypoint/src/try.kml"  
    	output_file = "/home/afonso/catkin_ws/src/waypoint/src/waypoints.txt"  
    
    	waypoints = parse_kml(kml_file)
    	save_to_txt(waypoints, output_file)
    	print(f"Coordenadas salvas em {output_file}")
    
   # realpath ...
   # python3 path ficheiro python
