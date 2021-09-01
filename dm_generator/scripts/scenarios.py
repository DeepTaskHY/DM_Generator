from abc import *
from typing import List, Dict
from proto.marshal.collections.maps import MapComposite

import os
from datetime import datetime
from distutils.util import strtobool
from xml.etree.ElementTree import Element, parse
from helpers import SCENARIO_PATH


class ScenarioBase(metaclass=ABCMeta):
    __root: Element = None

    def __init__(self, root: Element):
        self.__root = root

    @property
    def root(self) -> Element:
        return self.__root

    @property
    def name(self) -> str:
        return self.root.get('name')


class Parameter(ScenarioBase):
    __value = None

    @property
    def type(self) -> str:
        return self.root.get('type')

    @property
    def ref(self) -> List[str]:
        return self.root.get('ref').split('.')

    @property
    def value(self):
        return self.__value

    def set_content(self,
                    content: dict,
                    apis: Dict[str, object] = {}):

        ref = None

        if self.type == 'variable':
            ref = content

        elif self.type == 'time':
            ref = datetime.now()

        elif self.type == 'social_context':
            ref = content['social_context']

        elif self.type == 'dialogflow':
            api = apis['dialogflow']
            text = content['human_speech']
            ref = api.detect_intent_text(text).query_result

        if ref:
            for key in self.ref:
                if isinstance(ref, MapComposite) or isinstance(ref, dict):
                    if key in ref:
                        ref = ref[key]
                    else:
                        ref = None

                elif isinstance(ref, object):
                    ref = getattr(ref, key)

        self.__value = ref


class ParameterAssociationBase(ScenarioBase, metaclass=ABCMeta):
    __parameters: Dict[str, Parameter] = {}

    @property
    def parameter_values(self) -> Dict[str, str]:
        return {parameter_name: parameter.value for parameter_name, parameter in self.parameters.items()}

    @property
    def parameters(self) -> Dict[str, Parameter]:
        return self.__parameters

    @parameters.setter
    def parameters(self, value: Dict[str, Parameter]):
        self.__parameters = value


class ConditionBase(ParameterAssociationBase, metaclass=ABCMeta):
    @property
    def operator(self) -> str:
        return self.root.get('operator')

    @property
    def logic(self) -> str:
        return self.root.get('logic')

    @property
    @abstractmethod
    def result(self) -> bool:
        pass


class Condition(ConditionBase):
    @property
    def parameter(self) -> str:
        return self.root.get('parameter')

    @property
    def value(self) -> str:
        return self.root.get('value')

    @property
    def result(self) -> bool:
        parameter_value = self.parameter_values[self.parameter]

        if self.operator == 'exist':
            value = bool(strtobool(self.value))
            parameter_value = bool(parameter_value)
            return value == parameter_value

        elif self.operator == 'equal':
            return self.value == parameter_value

        elif self.operator == 'more':  # 이상
            value = float(self.value)
            parameter_value = float(parameter_value)
            return value <= parameter_value

        elif self.operator == 'below':  # 이하
            value = float(self.value)
            parameter_value = float(parameter_value)
            return value >= parameter_value

        elif self.operator == 'excess':  # 초과
            value = float(self.value)
            parameter_value = float(parameter_value)
            return value < parameter_value

        elif self.operator == 'under':  # 미만
            value = float(self.value)
            parameter_value = float(parameter_value)
            return value > parameter_value

        return False


class ConditionGroup(ConditionBase):
    @property
    def conditions(self) -> List[ConditionBase]:
        conditions = []

        if self.root:
            for child in self.root:
                if child.tag == 'condition':
                    condition = Condition(child)
                    condition.parameters = self.parameters
                    conditions.append(condition)

                elif child.tag == 'conditionGroup':
                    condition = ConditionGroup(child)
                    condition.parameters = self.parameters
                    conditions.append(condition)

        return conditions

    @property
    def result(self) -> bool:
        result = True
        logic = 'and'

        for condition in self.conditions:
            partial_result = condition.result

            if logic == 'and':
                result = result and partial_result
            elif logic == 'or':
                result = result or partial_result

            logic = condition.logic

        return result


class Dialog(ParameterAssociationBase):
    @property
    def format(self) -> Dict[str, str]:
        return {format.get('lang'): format.text for format in self.root.findall('./format')}

    @property
    def value(self) -> Dict[str, str]:
        return {lang: text.format(**self.parameter_values) for lang, text in self.format.items()}

    @property
    def conditions(self) -> ConditionGroup:
        conditions = ConditionGroup(self.root.find('./conditions'))
        conditions.parameters = self.parameters

        return conditions

    @property
    def result(self) -> bool:
        return self.conditions.result


class Intent(ScenarioBase):
    __parameters: Dict[str, Parameter] = {}

    @property
    def parameters(self) -> Dict[str, Parameter]:
        if not self.__parameters:
            parameters = self.root.findall('./parameters/parameter')
            self.__parameters = {root.get('name'): Parameter(root) for root in parameters}

        return self.__parameters

    def set_parameter_content(self,
                              content: dict,
                              apis: Dict[str, object] = {}):

        for parameter_name, parameter in self.parameters.items():
            parameter.set_content(content, apis)

    @property
    def correct_dialogs(self) -> List[Dialog]:
        dialogs = []

        for root in self.root.findall('./dialogs/dialog'):
            dialog = Dialog(root)
            dialog.parameters = self.parameters

            if dialog.result:
                dialogs.append(dialog)

        return dialogs


class Scenario(ScenarioBase):
    # Always return a new Intent instance
    def get_intent(self, intent_name: str) -> Intent:
        root = self.root.find(f'./intent[@name="{intent_name}"]')
        return Intent(root)


class XMLParser:
    def __init__(self, file_path: str):
        self.file_path = file_path
        self.tree = parse(file_path)

    @property
    def root(self) -> Element:
        return self.tree.getroot()


class ScenarioParser(XMLParser):
    __scenario_name: str = None
    __scenario_path: str = None

    def __init__(self, scenario_name: str):
        self.__scenario_name = scenario_name
        super(ScenarioParser, self).__init__(self.scenario_path)

    @property
    def scenario_name(self) -> str:
        return self.__scenario_name

    @property
    def scenario_path(self) -> str:
        if self.__scenario_path is None:
            self.__scenario_path = os.path.join(SCENARIO_PATH, f'{self.scenario_name}.xml')

        return self.__scenario_path

    # Always return a new Scenario instance
    def get_scenario(self) -> Scenario:
        return Scenario(self.root)
