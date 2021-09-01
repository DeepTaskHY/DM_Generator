import os
from google.cloud import dialogflow_v2 as dialogflow
from helpers import AUTHORIZATION_PATH


class DialogflowClient:
    __client: dialogflow.SessionsClient = None
    __project_id: str = None
    __session_id: str = None
    __key_file: str = None
    __key_path: str = None
    __language_code: str = None

    def __init__(self,
                 project_id: str,
                 session_id: str,
                 key_file: str,
                 language_code: str = 'ko'):

        self.__project_id = project_id
        self.__session_id = session_id
        self.__key_file = key_file
        self.__language_code = language_code
        self.authorize_credentials()

    @property
    def project_id(self):
        return self.__project_id

    @property
    def session_id(self):
        return self.__session_id

    @property
    def client(self):
        if self.__client is None:
            self.__client = dialogflow.SessionsClient()

        return self.__client

    @property
    def session(self):
        session = self.client.session_path(self.project_id, self.session_id)
        return session

    @property
    def key_path(self):
        if self.__key_path is None:
            self.__key_path = os.path.join(AUTHORIZATION_PATH, self.__key_file)

        return self.__key_path

    def authorize_credentials(self):
        os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = self.key_path

    @property
    def language_code(self):
        return self.__language_code

    def detect_intent_text(self, text: str):
        text_input = dialogflow.types.TextInput(text=text, language_code=self.language_code)
        query_input = dialogflow.types.QueryInput(text=text_input)

        response = self.client.detect_intent(request={
            'session': self.session,
            'query_input': query_input
        })

        return response
