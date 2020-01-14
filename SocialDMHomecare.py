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

os.environ["GOOGLE_APPLICATION_CREDENTIALS"]="socialrobot-hyu-xdtlug-eb5be21aa398.json" # homecare

'''
    Social Robot HYU
    Homecare Bot DM (generator) model
'''

# initialize the flask app
app = Flask(__name__)



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


def print_for_check(inout, data):
    print ''
    print '*' * 20 + ' DM module ' + inout +' ros massage ' + '*' * 20
    print json.dumps(data, ensure_ascii=False, indent=4)
    print ''


# START dialogflow_detect_intent_text
def detect_intent_texts(project_id, session_id, texts, language_code):
    import dialogflow_v2 as dialogflow
    session_client = dialogflow.SessionsClient()

    session = session_client.session_path(project_id, session_id)
    # print('Session path: {}\n'.format(session))

    for text in texts:
        text_input = dialogflow.types.TextInput(
            text=text, language_code=language_code)

        query_input = dialogflow.types.QueryInput(text=text_input)

        response = session_client.detect_intent(
            session=session, query_input=query_input)

    return response


def hello_bye_intent_detect(response, social_context, name, intent):
    if intent == "saying_hello":
        appellation = social_context['appellation']
        now = datetime.now()
        if response.query_result.parameters.fields['hello'].string_value: #check_information_sleep 시나리오
            if 11 >= now.hour >= 6:
                response.query_result.fulfillment_text = name+" "+appellation+", 좋은 아침이에요. 건강 상담이 필요하신가요?"
            elif now.hour <= 17:
                response.query_result.fulfillment_text = name+" "+appellation+", 좋은 오후예요. 건강 상담이 필요하신가요?"
            elif now.hour <= 21:
                response.query_result.fulfillment_text = name+" "+appellation+", 좋은 저녁이에요. 건강 상담이 필요하신가요?"
            else:
                response.query_result.fulfillment_text = name+" "+appellation+", 안녕하세요. 건강 상담이 필요하신가요?"
        else: # check_information_schedule 시나리오
            if 11 >= now.hour >= 6:
                response.query_result.fulfillment_text = name+" "+appellation+", 좋은 아침이에요. 잘 지내셨어요?"
            elif now.hour <= 17:
                response.query_result.fulfillment_text = name+" "+appellation+", 좋은 오후예요. 잘 지내셨어요?"
            elif now.hour <= 21:
                response.query_result.fulfillment_text = name+" "+appellation+", 좋은 저녁이에요. 잘 지내셨어요?"
            else:
                response.query_result.fulfillment_text = name+" "+appellation+", 잘 지내셨어요?"

    elif intent == "saying_good_bye":
        now = datetime.now()
        if 11 >= now.hour >= 6:
            response.query_result.fulfillment_text = response.query_result.fulfillment_text + " 좋은 하루 보내세요."
        elif now.hour <= 17:
            response.query_result.fulfillment_text = response.query_result.fulfillment_text + " 오후 잘 보내세요."
        elif now.hour <= 21:
            response.query_result.fulfillment_text = response.query_result.fulfillment_text + " 저녁 잘 보내세요."
        else:
            response.query_result.fulfillment_text = response.query_result.fulfillment_text + " 안녕히 가세요."


def medication_scenario_intent_detect(response, social_context, name, intent):
    if intent == "check_information_sleep":
        response.query_result.fulfillment_text = "간밤에 잘 주무셨어요?"

    elif intent == "transmit_information_reaction":
        if not response.query_result.parameters.fields['no'].string_value and not response.query_result.parameters.fields['negative'].string_value:
            response.query_result.fulfillment_text = "푹 주무셨다니 다행이에요."
        else:
            response.query_result.fulfillment_text = "잘 못 주무셨다니 속상해요."

    elif intent == "check_information_disease":
        disease = social_context['disease_name']
        response.query_result.fulfillment_text = disease+"은 좀 어떠세요?"

    elif intent == "check_information_meal":
        dialog = " 식사는 뭐 드셨나요?"
        if response.query_result.parameters.fields['positive'].string_value != u'':
            response.query_result.fulfillment_text = "좋은 소식이네요."+dialog
        elif response.query_result.parameters.fields['negative'].string_value != u'':
            response.query_result.fulfillment_text = "안 좋은 소식이네요."+dialog

    elif intent == "transmit_information_disease_advise":
        disease = social_context['disease_name']
        advice = social_context['disease_advice']
        if response.query_result.parameters.fields['meal'].string_value:
            response.query_result.fulfillment_text = "잘 하셨어요. "+disease+"에 "+advice+" 중요한 거 아시죠?"
        else:
            response.query_result.fulfillment_text = "조심하셔야 해요. "+disease+"에 "+advice+" 중요한 거 아시죠?"

    elif intent == "check_information_health":
        task = social_context['task']
        if response.query_result.parameters.fields['negative'].string_value == u'':
            response.query_result.fulfillment_text = "다행이에요. "+task+"은 하셨나요?"
        else:
            response.query_result.fulfillment_text = "주의해주세요. "+task+"은 하셨나요?"

    elif intent == "transmit_information_health_advice":
        take = social_context['medicine_schedule']
        if response.query_result.parameters.fields['negative'].string_value == u'':
            response.query_result.fulfillment_text = "잘 챙기셨네요. "+take+"하셔야 해요."
        else:
            response.query_result.fulfillment_text = "다음에는 빠뜨리시면 안 돼요. "+take+"하셔야 해요."


def hospital_schedule_detect_intent(response, social_context, name, intent):
    if intent == "check_information_schedule":
        visit = social_context['visit_place']
        if not response.query_result.parameters.fields['no'].string_value and not response.query_result.parameters.fields['negative'].string_value:
            response.query_result.fulfillment_text = "잘 지내셨다니 다행이에요."
        else:
            response.query_result.fulfillment_text = "잘 못 지내셨다니 슬퍼요."
        dialog = " 오늘 "+visit+" 방문하시는 날이었죠?"
        response.query_result.fulfillment_text = response.query_result.fulfillment_text + dialog

    elif intent == "transmit_information_disease_regard":
        disease = social_context['disease_name']
        status = social_context['disease_status']
        appellation = social_context['appellation']
        dialog = appellation+" "+disease+"이 "
        if status == "negative":
            dialog = dialog + "나빠지셨다고 들었어요."
        elif status == "positive":
            dialog = dialog + "좋아지셨다고 들었어요."

        if response.query_result.parameters.fields['positive'].string_value != u'':
            response.query_result.fulfillment_text = "잘 하셨어요. " + dialog
        else:
            response.query_result.fulfillment_text = "꼭 가셔야 해요. " + dialog

    elif intent == "transmit_information_medicine":
        disease = social_context['disease_name']
        status = social_context['disease_status']
        appellation = social_context['appellation']
        dialog = appellation+" "+disease+"이 "
        if status == "negative":
            response.query_result.fulfillment_text = dialog + "나빠지셔서 드시는 약이 바뀌었어요."
        elif status == "positive":
            response.query_result.fulfillment_text = dialog + "좋아지셔서 드시는 약이 바뀌었어요."



def ros_callback(msg):
    if msg.data != '':
        # convert ros message to json
        ros_input = json.loads(msg.data, encoding='utf-8')

        if "dialog" == ros_input['header']['target'][0]:
            print_for_check(" Input ", ros_input)

            name = ros_input['dialog_generation']['name']
            intent = ros_input['dialog_generation']['intent']
            human_speech = ros_input['dialog_generation']['human_speech']
            id = ros_input['dialog_generation']['id']
            social_context = ros_input['dialog_generation']['social_context']
            lan_code = isHangul(human_speech)

            if lan_code == "k":
                response = detect_intent_texts('socialrobot-hyu-xdtlug', 'hyusocialdmgenerator', [human_speech], 'ko') # homecare ko
            else:
                response = "영문버전은 아직 준비되지 않았어요."

            hello_bye_intent_detect(response, social_context, name, intent)
            medication_scenario_intent_detect(response, social_context, name, intent)
            hospital_schedule_detect_intent(response, social_context, name, intent)

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
