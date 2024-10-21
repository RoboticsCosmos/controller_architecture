import os
import fnmatch
import json
import re

"""
Script to check consistency of all metamodels. These checks are done if it is a dictionary:
1. it has an URI associated with a key same as the filename
2. the URI ends with the filename followed by "#"
3. if key is a dictionary, then the key "@id" has the filename and the key in the format "filename:key"
4. if key is not a dictionary, then the key has the format "filename:key"
5. populate namespaces.py with all the keys under respective classes
"""

meta_models_path = os.path.join(os.path.dirname(__file__), "../metamodels/")

my_dict = dict()
for root, dirs, files in os.walk(meta_models_path):

    for filename in files:

        if fnmatch.fnmatch(filename, "*.jsonld"):
            filename_without_extension = filename.split(".")[0]
            # read the file as a dictionary and check if it contains a key "@context"
            try:
                with open(os.path.join(root, filename), "r") as f:
                    data = json.load(f)
                    my_list = []

                    if "@context" in data:
                        # loop trough the keys of the dictionary "@context" and check if they are dictionaries
                        for key in data["@context"]:
                            skip = False

                            if isinstance(data["@context"][key], dict):
                                a, b = data["@context"][key]["@id"].split(":")

                            elif not data["@context"][key].startswith("http"):
                                a, b = data["@context"][key].split(":")

                            else:
                                if filename_without_extension != key:
                                    print("[",filename,"]", "along with URI not found in @context")
                                end_text = filename_without_extension + "#"
                                URI_text = data["@context"][key]
                                if not data["@context"][key].endswith(end_text):
                                    print(filename, "URI does not end with: ",filename,"#")
                                skip = True
                                my_list.append(URI_text)

                            # check if the key "@id" has the filename and the key
                            if not skip:
                                my_list.append(key)

                                if a != filename_without_extension:
                                    print("[",filename,"] LHS:", " filename not found in key: ", key)

                                if b != key:
                                    print("[",filename,"] RHS:", key, " not found")

                    my_dict[filename_without_extension] = my_list
                    # print("\n LIST: ",my_list , "\n")

            except json.JSONDecodeError as e:
                print(filename, " is not a dictionary. Skipping...")
# print("\n Dictionary with all keys: ", my_dict)


def generate_python_script(classes_dict, output_file_name):

# if output_file_name != "namespaces.py":

    with open(output_file_name, "w") as f:
        f.write("from rdflib.namespace import DefinedNamespace, Namespace\n")
        f.write("from rdflib.term import URIRef\n")

        for class_name, class_values in classes_dict.items():
            class_name_upper = class_name.upper()
            f.write(f"\n\nclass {class_name_upper}(DefinedNamespace):\n\n")
            for value in class_values:
                if value.startswith("http"):
                    URI_text = value
                    continue
                f.write(f"    {value}: URIRef\n")

            f.write(f"\n    _NS = Namespace(\"{URI_text}\")")
# else:
#     print("Please provide a different output file name than 'namespaces.py'")


namespeces_output_path = (
    "/home/sawantk/freddy_ws/controller_architecture_thesis/synthesis/namespaces.py"
)
generate_python_script(my_dict, namespeces_output_path)

"""
algorithm.jsonld
error.jsonld
functions.jsonld
controller.jsonld
abag.jsonld
pid_controller.jsonld
p_controller.jsonld
impedance_controller.jsonld
estimators.jsonld
monitor.jsonld
plan.jsonld
"""
