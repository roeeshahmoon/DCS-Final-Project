# Light Sources and Objects Detector System

![light_detector](Images/light_detector.jpeg)

## Target of the project 
 The purpose of the project is to implement, design and demonstrate an embedded system based on the MCU PC and capable of detecting light sources and tracking objects in space 
 using a number of components such as the ultrasonic distance meter, sonic LDR light sensors and a servo motor. The main goal is to create a scanning mechanism that covers an area 
 of 180 degrees to collect data from the surrounding environment just like.
 This innovative system can help promise for applications in civilian and military security systems, environmental monitoring, and various other scenarios requiring efficient 
 light source detection and object monitoring.

## Features

### Light source detection:
 In the project we will develop a reliable method for detecting light sources within the scanning area using the LDR light sensors. The system should be able to differentiate 
 between different light intensities and accurately determine the positions of the light sources, by calibration to be carried out in advance.

### Monitoring:
 Which determines precise control of the angular movement of the servo motor using a PWM signal from the MCU. The servo motor should be able to detect the intended area of 180 
 degrees in continuous motion. and accurate


### Data processing:
 Development of algorithms for processing the data collected from the ultrasonic distance meter and the LDR sensors. By processing the sensor data to create a comprehensive 
 understanding of the environment, that is, identifying the distance of objects and light sources.

### Computer GUI user interface to design:
 An intuitive user interface with the mouse to select the mode desired by the user and in addition we will present the results of the scan in time. In addition, the interface will 
 display relevant information regarding the distance from detected light beams and object distances by the Matplotlib book and Numpy.

### Precision Calibration: 
 Ensuring the accuracy and reliability of the system by calibrating the sensors and servo motor to reduce measurement errors leads to improved performance.

### Mobility:
 Mobility to create a compact system that can be easily deployed and installed in different environments for monitoring and data collection applications.

## Usage
It is designed to efficiently perform matrix multiplication using a systolic array architecture, which enables a highly parallel
and pipelined computational structure to achieve high throughput and reduced latency compared to traditional sequential methods.

### Technology Stack

- Design stage written in Verilog using HDL
- Verification Stage written in Python Scripts, System Verilog and ModelSim

### Instantiation

To use the Matrix Multiplication Accelerator in your Verilog design, instantiate the `matmul_pkg` module with appropriate parameters with the correct constrains

### Configuration

- `BUS_WIDTH`: Width of the data bus connecting to the accelerator.
- `DATA_WIDTH`: Width of each data element in the matrices.
- `ADDR_WIDTH`: Width of the address bus for memory access.
- `SP_NTARGETS`: Number of targets in the scratchpad memory.

### Ports

- **Inputs**:
  - `clk_i`: Clock input.
  - `rst_ni`: Reset Negative input (active low).
  - `psel_i`: Peripheral select input.
  - `penable_i`: Peripheral enable input.
  - `pwrite_i`: Write/Read enable input.
  - `pstrb_i`: Byte enable input.
  - `pwdata_i`: Write data input.
  - `paddr_i`: Address input.

- **Outputs**:
  - `pready_o`: Ready output.
  - `pslverr_o`: Slave error output.
  - `prdata_o`: Read data output.
  - `busy_o`: Busy output.
  - `done_o`: Done output.

### Flow Chart

![Flow_Chart](/doc/Images/Flow_Chart.png)

### Control and Status

The accelerator operates based on control signals provided through the `psel_i`, `penable_i`, `pwdata_i` and `paddr_i` inputs from APB.
Status of the accelerator operation is indicated through the `busy_o` and `done_o` outputs.

## Verification 

### Test Bench Block Diagram

![Test_Bench](/doc/Images/Test_Bench.png)

We read all the data for the testbench from file `Bus_File.txt`, we randomize all the data on the python script `golden.py` using `python random moudle`.

The stimulus read instructions from  `Bus_File.txt` and generate APB master to write this data into the design.
 We wrote those data into files:
 `Param_File.txt`,
 `Mat_A.txt`,
 `Mat_B.txt`,
 `Mat_Res.txt`,
 `Flags_Res.txt`,
 `SP.txt`.

### Test Example

 ![Test_Exa](/doc/Images/Test_Exa.png)

In the end we are comparing between the result files from DUT `MAT_RES_DUT.txt` and `FLAGS_RES_DUT.txt` to golden script. 
We print to the screen how much hits and miss we have.


## Authors

- Roee Shahmoon
- Noam Klainer 
