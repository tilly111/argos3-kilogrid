# ARGoS3-Kilogrid Plug-in

In the following we give a short overview of the Kilogrid plug-in for the simulation environment 
ARGoS3. 

## Setup

First, install ARGoS3 (https://github.com/ilpincy/argos3) and the Kilobot plug-in 
(https://github.com/ilpincy/argos3-kilobot).

Second, renew the link to ARGoS (maybe you have a different path!).
```
cd ARGoS_simulation/
rm argos3
cd ..
ln -s ~/Programs/argos3-kilobot/src/ ARGoS_simulation/argos3
```

Finally, you can run the sample experiment, which does nothing besides starting the simulation,
by executing
```
cd argos3-kilogrid
sh test.sh
```

## Running Experiments
You can build the template
```
cd argos3-kilogrid
mkdir build
cd build
cmake ../ARGoS_simulation
cd ..
```

The `test.sh` makes the template and strats the sample experiment. 
```
sh test.sh
```

## Documentation

### Overview

#### experiment 
- this folder contains the `.argos` files, which set up the experiment environment 
- here you have to adjust the paths for controller and setup files (indicated with TODO)

#### loop_functions
- you can add the `.kconf` files in `loop_functions/kilogrid_conf_files/`
- implements the Kilogrid functionality (discussed later) 

#### data
- here you can store the generated data files from the simulation

#### behaviours
- `agent.h`, here you can add the infrared message types and the data you want to track
- `agent_template.c`, controller for the Kilobot 





### Where to implement what?!
You have to implement the following methodes in `loop_functions/kilogrid_template.cpp`:
- `void CKilogrid::setup(int x, int y)`, is the setup function of the individual module.
- `CKilogrid::loop(int x, int y)`, implements the loop which gets executed by every module every cycle.
- `CKilogrid::IR_rx(int x, int y, IR_message_t *m, cell_num_t c, distance_measurement_t *d,
  uint8_t CRC_error)`, infra red callback
- `CKilogrid::CAN_rx(int x, int y, CAN_message_t *m)`, CAN message callback 

Keep in mind that you also have to initialise the variables you use in the individual modules in the 
`kilogrid_template.h` (indicated with TODO).

The robot controller can be implemented in the `behaviours/agent_template.c`. 



### TODOS
I'm glad you use this template. If you find any bugs or something you want to have added, just 
write me an email: till.aust[at]student.uni-luebeck.de.