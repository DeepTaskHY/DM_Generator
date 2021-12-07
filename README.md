# 1. [M2-6] Dialogue Generator

## 2. Package summery 

The dialogue generator is an agent that generates the robot utterance in human-robot interaction. When the user utterance and user information taken as input values, the robot utterance is generated using the given information. The user information is obtained from the Task Manager through ROS protocol. After the robot utterance created, the generated robot utterance is forwarded to the Task Manager again through the ROS protocol.

- 2.1 Maintainer status: maintained
- 2.2 Maintainer: Yuri Kim ([yurikim@hanyang.ac.kr]()), Eunsoo Lee ([eunsoogi@hanyang.ac.kr]())
- 2.3 Author: Yuri Kim ([yurikim@hanyang.ac.kr]())
- 2.4 License (optional): 
- 2.5 Source git: https://github.com/DeepTaskHY/DM_Generator

## 3. Overview

When creating the robot utterance, we define several tasks that consider sociality for natural dialogue with the user. Each task generates various utterances according to social information(e.g. user profile, health information, schedule information). The social information is updated by the Knowledge manager that manages knowledge in the form of ontology. 

## 4. Environments

- [ros:noetic](https://hub.docker.com/layers/ros/library/ros/noetic/images/sha256-c1565b2b554d775f1fb2fde93d1aaf76554a6a98d06f10432b0dd4ddd5d6a11c)
- Python 3.7+

## 5. Quick start

### 5.1 Start the module

- Go to [Author's Google Drive](https://drive.google.com/file/d/1Tya9XQrtlAv393xh8D_5MYfBAta15quz/view?usp=sharing) and download the JSON file (authorization key), and put it in the [authorization folder](dm_generator/keys/).
- After copying the [configuration.json.example](dm_generator/configuration.json.example) to the [configuration.json](dm_generator/configuration.json), edit the authentication key file name, etc.

### 5.2 Test the module

To test this module, you can execute the following command line.

**homecare**

```shell
$ docker-compose run -e scenario_name=homecare dm-default  # or
$ docker-compose run -e scenario_name=homecare dm-linux  # or
$ roslaunch dm_generator dm_launcher.launch SCENARIO_NAME:=homecare
```

**reception**

```shell
$ docker-compose run -e scenario_name=reception dm-default  # or
$ docker-compose run -e scenario_name=reception dm-linux  # or
$ roslaunch dm_generator dm_launcher.launch SCENARIO_NAME:=reception
```

**testbench**

```shell
$ python dm_generator/tests/testbench.py [-h] --testbench {homecare,reception} [--starts STARTS] [--delay DELAY]
```

## 6. Input/Subscribed Topics

```json
{
    "header": {
        "id": 1,
        "timestamp": "1563980674.262554407",
        "source": "planning",
        "target": ["dialog"], 
        "content": ["dialog_generation"], 
    }, 
    "dialog_generation": {
        "intent": "transmit_information_health_advice",
        "human_speech": "안 먹었어.",
        "social_context": {
            "name": "이병현",
            "medicine_schedule": "식후 30분 후 복용"
        }
    }
}
```

○ header (header/taskExecution): contain information about published time, publisher name, receiver name. 

- id: id
- timestamp: published time
- source: publish the module name
- target: receive module name
- content: role of this ROS topic name

○ dialog_generation (dialog_generation/taskExecution): contain required information to generate robot utterance sentence. (Import from Knowledge Manager) 

- intent: sub-task name
- social_context: required information to generate a robot utterance sentence. (Import from knowledge manager)

## 7. Output/Published Topics

```json
{
    "header": {
        "id": 1,
        "timestamp": "1563980674.262554407",
        "source": "dialog",
        "target": ["planning"],
        "content": ["dialog_generation"]
    },
    "dialog_generation": {
        "dialog": "다음에는 빠뜨리시면 안 돼요. 식후 30분 후 복용하셔야 해요.",
        "result": "completion"
    }
}
```

○ header (header/taskExecution): contain information about published time, publisher name, receiver name.  

- id: id
- timestamp: published time  
- source: publish module name  
- target: receive module name  
- content: role of this ROS topic name  

○ dialog_generation (dialog_generation/taskExecution): contain generated robot speech sentence, id, result.  

- dialog: generated robot speech  
- result: task progress result

## 8. Related Applications (Optional)

None

## 9. Related Publications (Optional)

None
