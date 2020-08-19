# 1. [M2-6] Dialogue Generator

## 2. Package summery 

The dialogue generator is an agent that generates the robot utterance in human-robot interaction. When the user utterance and user information taken as input values, the robot utterance is generated using the given information. The user information is obtained from the Task Manager through ROS protocol. After the robot utterance created, the generated robot utterance is forwarded to the Task Manager again through the ROS protocol.

- 2.1 Maintainer status: maintained
- 2.2 Maintainer: Yuri Kim, [yurikim@hanyang.ac.kr]()
- 2.3 Author: Yuri Kim, [yurikim@hanyang.ac.kr]()
- 2.4 License (optional): 
- 2.5 Source git: https://github.com/DeepTaskHY/DM_Generator

## 3. Overview

When creating the robot utterance, we define several tasks that consider sociality for natural dialogue with the user. Each task generates various utterances according to social information(e.g. user profile, health information, schedule information). The social information is updated by the Knowledge manager that manages knowledge in the form of ontology. 

## 4. Hardware requirements

None

## 5. Quick start 

### 5.1 Install dependency:

**requirements**  

    $ pip install -r requirements.txt  
    $ pip install --upgrade google-auth-oauthlib  
    $ pip install --upgrade pyasn1-modules  

**ros-melodic**

    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    $ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    $ sudo apt-get update  
    $ sudo apt-get install ros-melodic-desktop-full  
    $ sudo rosdep init  
    $ rosdep update  

### 5.2 Start the module

- If you don't have permission, go to [Author's Google drive](https://drive.google.com/file/d/1Tya9XQrtlAv393xh8D_5MYfBAta15quz/view?usp=sharing) and download the JSON file(auth key). 
  Add the path of the downloaded JSON file to line 16 of 'SocialDMReception.py '. 
  You can start this module by running the 'SocialDMReception.py' file.

### 5.3 Test the module

To test this module, you can execute the following command line. This command sends a ROS message.

**homecare**

```
$ cd dm_generator/
$ roslaunch launch/dm_homecare.launch 
```

**reception**

```
$ cd dm_generator/
$ roslaunch launch/dm_reception.launch
```



## 6. Input/Subscribed Topics

```
{
    'header': {
        'timestamp': '1563980674.262554407', 
        'target': ['dialog'], 
        'content': ['dialog_generation'], 
        'source': 'planning'
    }, 
        'dialog_generation': {
        'name': '이병현', 
        'intent': 'transmit_information_health_advice', 
        'id': 177, 
        'human_speech':'응 먹었지.', 
        'social_context': {'take_medicine_schedule': '식후 30분 후'}
    }
}
```

○ header (header/taskExecution): contain information about published time, publisher name, receiver name. 

- timestamp: published time 
- target: receive module name 
- content: role of this ROS topic name 
- source: publish the module name 

○ dialog_generation (dialog_generation/taskExecution): contain required information to generate robot utterance sentence. (Import from Knowledge Manager) 

- intent: sub-task name 
- id: id 
- name: user name 
- gender: user gender 
- appellation: This is the name that the robot uses when calling the user. It is determined by the age of the user. 
- visit_history: Whether the user has visited the hospital. 
- medical_record: User's disease information 
- social_context: required information to generate a robot utterance sentence. (Import from database) 
- take_medicine_schedule: User's medication time 

## 7. Output/Published Topics

```
{
    "header": {
        "timestamp": "%i.%i" % (current_time.secs, current_time.nsecs),
        "source": "dialog",
        "target": ["planning"],
        "content": ["dialog_generation"]
    },
    "dialog_generation": {
        "id": id,
        "dialog": dialog,
        "result": "completion"
    }
}
```

○ header (header/taskExecution): contain information about published time, publisher name, receiver name.  

- timestamp: published time  
- source: publish module name  
- target: receive module name  
- content: role of this ROS topic name  

○ dialog_generation (dialog_generation/taskExecution): contain generated robot speech sentence, id, result.  

- id: id  
- dialog: generated robot speech  
- result: task progress result  

## 8. Parameters

: There are two categories of parameters that can be used to configure the Dialog Generator module: dialogflow client, fulfillment.  

### 8.1 dialogflow client parameters

-  ~GOOGLE_APPLICATION_CREDENTIALS (string, default: None): The path where the authentication key(JSON file) is stored.  
-  ~project_id (string, default: ‘socialrobot-hyu-xdtlug’): Project id of Dialogflow.  
-  ~session_id (string, default: None): It can be a random number or some type of user identifier(preferably hashed). The length of the session ID must not exceed 36 bytes.  
-  ~texts (string, default: None): User’s speech sentence.  
-  ~language_code (string, default: ko): The language of this conversational query. (ko: Korean, en: English)  

### 8.2 fulfillment parameters 

- ~IP_address (string, default:“localhost”): IP address  
- ~port_num (string, default:5000): port number  

## 9. Related Applications (Optional)

None

## 10. Related Publications (Optional)

-  Dialogflow API official document[Website]. (2019, Aug 22). https://cloud.google.com/dialogflow/docs/reference/rest/v2/projects.agent.sessions/detectIntent  
-  ROS Kinetic installation instructions[Website]. (2019, Aug 22). http://wiki.ros.org/kinetic/Installation  
