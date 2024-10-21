import os
import json


def get_all_jsonld_files(root_dir):
    """Recursively get all JSON-LD files in the given directory."""
    jsonld_files = []
    for dirpath, _, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.endswith(".jsonld"):
                jsonld_files.append(os.path.join(dirpath, filename))
    return jsonld_files


def check_unique_keys(jsonld_files):
    """Check if all JSON-LD files have unique keys and print repeated keys."""
    all_keys = {}
    repeated_keys = {}

    for file_path in jsonld_files:
        with open(file_path, "r", encoding="utf-8") as file:
            file_name = os.path.basename(file_path)
            try:
                data = json.load(file)
            except json.JSONDecodeError:
                print(f"Failed to parse {file_name} as JSON.")
                continue

            for key in data["@context"].keys():
                if key in all_keys:
                    repeated_keys.setdefault(key, set()).update(
                        [all_keys[key], file_name]
                    )
                else:
                    all_keys[key] = file_name

    if repeated_keys:
        print("Repeated keys found:")
        for key, files in repeated_keys.items():
            print(f"Key: {key} found in files: {', '.join(files)}")
    else:
        print("All keys are unique across JSON-LD files.")


def main():
    root_dir = "/home/sawantk/freddy_ws/controller_architecture_thesis/metamodels/"
    jsonld_files = get_all_jsonld_files(root_dir)
    if not jsonld_files:
        print("No JSON-LD files found.")
        return

    check_unique_keys(jsonld_files)


if __name__ == "__main__":
    main()
