## Controller_architecture for robots


## OS
- The code is tested on Ubuntu 22

## Dependencies:
- rdflib
- jinja2
- kdl_parser
- orocos_kinematics_dynamics
- StringTemplate

## Setup

### Step 1

- clone the repository to a workspace 

    ```bash
    mkdir -p ~/ctr_arch_ws/src && cd ~/ctr_arch_ws/src

    git clone https://github.com/RoboticsCosmos/controller_architecture.git
    ```

### Step 3

- Install the dependencies: kdl_parser, yaml-cpp

    ```bash
    sudo apt-get install libkdl-parser-dev libyaml-cpp-dev 
    ```
- clone and install StringTeplate in the same workspace using https://github.com/antlr/stringtemplate4/tree/master


## Usage

### Use Case 1

- Run the following commands from the src/controller_architecture/ path
    
    ```bash
    cd gen/
    python runner_uc1.py

    cd ../code_generator
    make uc1 > ../gen/generated_code.cpp

    ```

### Use Case 2 - PID

- Run the following commands from the src/controller_architecture/ path
    
    ```bash
    cd gen/
    python runner_uc2_pid.py

    cd ../code_generator
    make uc2_pid > ../gen/generated_code.cpp

    ```

### Use Case 2 - ABAG

- Run the following commands from the src/controller_architecture/ path
    
    ```bash
    cd gen/
    python runner_uc2_abag.py

    cd ../code_generator
    make uc2_abag > ../gen/generated_code.cpp

    ```

### executing generated code

- Compile the workspace (from the src/controller_architecture/ path)

    ```bash
    cmake -Bbuild .

    ```

- Build the workspace (from the src/controller_architecture/ path)

    ```
    cmake --build build

    ```

- Execute the generated code (from the src/controller_architecture/ path)
  
    ```
    ./build/gen/generated_code_binary

    ```
