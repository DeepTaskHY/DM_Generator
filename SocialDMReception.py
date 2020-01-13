# -*- coding: utf-8 -*-
import sys
import yaml
import json
import time
import threading
import os
import re
import rospy
from std_msgs.msg import String
from flask import Flask, request, make_response, jsonify
import dialogflow
from datetime import datetime

reload(sys)
sys.setdefaultencoding('utf-8')

os.environ[
    "GOOGLE_APPLICATION_CREDENTIALS"] = "socialrobot-hyu-reception-nyla-176b1cd7164b.json"  # reception

'''
    Social Robot HYU
    Homecare Bot DM (generator) model
'''

# initialize the flask app
app = Flask(__name__)


def print_for_check(inout, data):
    print ''
    print '*' * 20 + ' DM module ' + inout +' ros massage ' + '*' * 20
    print json.dumps(data, ensure_ascii=False, indent=4)
    print ''


def isHangul(text):
    #Check the Python Version
    pyVer3 =  sys.version_info >= (3, 0)

    if pyVer3 : # for Ver 3 or later
        encText = text
    else: # for Ver 2.x
        if type(text) is not unicode:
            encText = text.decode('utf-8')
        else:
            encText = text

    hanCount = len(re.findall(u'[\u3130-\u318F\uAC00-\uD7A3]+', encText))
    if hanCount > 0:
        code = "k"
    else:
        code = "e"
    return code


# make response json
def make_response_json(id, dialog):
    final_response = {
        'header': {
            'content': 'dialog_generation',
            'source': 'dialog',
            'target': 'planning',
            'timestamp': str(time.time())
        },
        'dialog_generation': {
            'id': id,
            'dialog': dialog,
            'result': 'completion'
        }
    }
    return final_response


# START dialogflow_detect_intent_text
def detect_intent_texts(project_id, session_id, texts, language_code):
    import dialogflow_v2 as dialogflow
    session_client = dialogflow.SessionsClient()

    session = session_client.session_path(project_id, session_id)

    for text in texts:
        text_input = dialogflow.types.TextInput(
            text=text, language_code=language_code)

        query_input = dialogflow.types.QueryInput(text=text_input)

        response = session_client.detect_intent(
            session=session, query_input=query_input)

    return response


def welcome_bye_intent_detect(response, social_context, name, intent):
    if intent == "saying_welcome":
        appellation = social_context['appellation']
        now = datetime.now()
        if 11 >= now.hour >= 6:
            response.query_result.fulfillment_text = name+" "+appellation+", 좋은 아침이에요. " + response.query_result.fulfillment_text
        elif now.hour <= 16:
            response.query_result.fulfillment_text = name+" "+appellation+", 좋은 오후예요. " + response.query_result.fulfillment_text
        elif now.hour <= 21:
            response.query_result.fulfillment_text = name+" "+appellation+", 좋은 저녁이에요. " + response.query_result.fulfillment_text
        else:
            response.query_result.fulfillment_text = name+" "+appellation+", 안녕하세요. " + response.query_result.fulfillment_text

    elif intent == "saying_good_bye":
        now = datetime.now()
        if 11 >= now.hour >= 6:
            response.query_result.fulfillment_text = response.query_result.fulfillment_text + " 좋은 하루 보내세요."
        elif now.hour <= 16:
            response.query_result.fulfillment_text = response.query_result.fulfillment_text + " 오후 잘 보내세요."
        elif now.hour <= 21:
            response.query_result.fulfillment_text = response.query_result.fulfillment_text + " 저녁 잘 보내세요."
        else:
            response.query_result.fulfillment_text = response.query_result.fulfillment_text + " 안녕히 가세요."


def ros_callback(msg):
    if msg.data != '':
        ros_input = json.loads(msg.data, encoding='utf-8')

        if ros_input['header']['target'][0] == "dialog":
            print_for_check(" Input ", ros_input)

            name = ros_input['dialog_generation']['name']
            intent = ros_input['dialog_generation']['intent']
            human_speech = ros_input['dialog_generation']['human_speech']
            id = ros_input['dialog_generation']['id']
            social_context = ros_input['dialog_generation']['social_context']
            lan_code = isHangul(human_speech)

            if lan_code == "k":
                response = detect_intent_texts('socialrobot-hyu-reception-nyla', 'hyusocialdmgenerator', [human_speech], 'ko') # homecare ko
            else:
                response = "영문버전은 아직 준비되지 않았어요."

            welcome_bye_intent_detect(response, social_context, name, intent)

            if intent == "check_information_disease":
                disease = social_context['disease_name']
                status = social_context['disease_status']
                appellation = social_context['appellation']
                if status == 'positive':
                    response.query_result.fulfillment_text = "네, "+name+" "+appellation+". 몇 주간 식단 조절을 하시더니 "+disease+" 경과가 좋아지셨다고 하더라고요. 기쁘시겠어요."
                elif status == 'negative':
                    response.query_result.fulfillment_text = "네, "+name+" "+appellation+". 요즘 "+disease+" 경과가 안 좋아지셨다고 하더라고요. 걱정스러우시겠어요."

            elif intent == "transmit_information_pharmacy":
                opening_hour = social_context['opening_hour']
                place = social_context['place']
                treat = social_context['treat']
                response.query_result.fulfillment_text = "여기 "+treat+"입니다. 저희 센터 옆 "+place+"에서 받아가실 수 있고, "+opening_hour+"까지 운영해요. 다른 도움이 필요하신가요?"


            if intent == "transmit_information_disease_advice":
                disease = social_context['disease_name']
                description = social_context['disease_description']
                symptom = social_context['disease_symptom']
                prevent = social_context['prevent']
                response.query_result.fulfillment_text = "네, "+disease+"은 "+description+"입니다. 증상은 "+symptom+" 등이 있어요. "\
                                                         +prevent+"으로 "+disease+"을 예방할 수 있어요. 저도 작년에 컴퓨터 바이러스를 잡았는데 플래쉬 드라이브를 더 조심해야겠어요. 하하하."

            elif intent == "check_information_reservation":
                treat = social_context['treat']
                response.query_result.fulfillment_text = "저희 클리닉에서 "+treat+"을 받으실 수 있어요. 예약을 도와드릴까요?"

            elif intent == "transmit_information_center_schedule":
                if response.query_result.parameters.fields['negative'].string_value != u'':
                    response.query_result.fulfillment_text = "네, 알겠습니다."
                else:
                    date = social_context['date']
                    avail_times = social_context['available_time']
                    response.query_result.fulfillment_text = date+" "+avail_times+"에 예약하실 수 있어요. 시간 괜찮으세요?"

            elif intent == "check_information_alarm":
                reserve_time = response.query_result.parameters.fields['time'].string_value.encode("utf-8")[11:16]
                avail_times = social_context['available_time'].split(', ')
                right_time = False
                for avail_time in avail_times:
                    hour = avail_time[0:2]
                    min = avail_time[3:5]
                    if int(hour) < 12:
                        hour2 = str(int(hour) + 12)
                    else:
                        if int(hour)-12 < 10:
                            hour2 = '0'+str(int(hour)-12)
                        else:
                            hour2 = str(int(hour)-12)
                    avail_time2 = hour2+":"+min
                    if reserve_time == avail_time or reserve_time == avail_time2:
                        right_time = True
                        break

                if not right_time:
                    response.query_result.fulfillment_text = "죄송해요. 그 시간에는 예약하실 수 없어요."
                if response.query_result.parameters.fields['available_time'].string_value != u'' or \
                        response.query_result.parameters.fields['negative'].string_value != u'':
                    response.query_result.fulfillment_text = "죄송해요. 지금은 그 시간에만 가능해요."

            elif intent == "transmit_information_reservation":
                date = social_context['date']
                time = response.query_result.parameters.fields['time'].string_value.encode("utf-8")
                if response.query_result.parameters.fields['negative'].string_value != u'':
                    response.query_result.fulfillment_text = "알겠어요. 그럼 " + date + " " + time + "에 뵙겠습니다."
                else:
                    response.query_result.fulfillment_text = "홈케어 로봇에게 예약을 알리는 메시지를 보냈어요. 그럼 " + date + " " + time + "에 뵙겠습니다."

            robot_dialog = response.query_result.fulfillment_text.encode("utf-8")

            final_output = make_response_json(id, robot_dialog)

            task_completion_pub = rospy.Publisher('/taskCompletion', String, queue_size=10)
            task_completion_pub.publish(json.dumps(final_output, ensure_ascii=False, indent=4))

            print_for_check(" Output", final_output)
            print("="*120)


def run_subscriber():
    threading.Thread(target=lambda: rospy.init_node('dm_node', disable_signals=True)).start()
    rospy.Subscriber('/taskExecution', String, ros_callback)


# default route
@app.route('/')
def index():
    msg = String()
    msg.data = 1
    return 'Social Robot Dialogflow HYU Homecare'


# create a route for webhook
@app.route('/reception', methods=['GET', 'POST'])
def webhook():
    run_subscriber()
    return run_subscriber()


if __name__ == '__main__':
    run_subscriber()
    app.run()
