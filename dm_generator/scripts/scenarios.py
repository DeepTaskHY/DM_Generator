from abc import *
from typing import List, Dict
from proto.marshal.collections.maps import MapComposite

import os
from datetime import datetime
from distutils.util import strtobool
from xml.etree.ElementTree import Element, parse
from helpers import SCENARIO_PATH, reverse_time


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
    __partial: List[int] = []

    @property
    def type(self) -> str:
        return self.root.get('type')

    @property
    def ref(self) -> List[str]:
        return self.root.get('ref').split('.')

    @property
    def partial(self) -> List[int]:
        if not self.__partial and self.root.get('partial'):
            self.__partial = [int(num) for num in self.root.get('partial').split(':')]

        return self.__partial

    @property
    def split(self) -> str:
        return self.root.get('split')

    @property
    def value(self):
        return self.__value

    def set_content(self, content: dict):
        ref = None

        if self.type == 'variable':
            ref = content

        elif self.type == 'time':
            ref = datetime.now()

        elif self.type == 'social_context':
            ref = content['social_context']

        elif self.type == 'dialogflow':
            ref = content['dialogflow'].query_result

        if ref:
            for key in self.ref:
                if isinstance(ref, MapComposite) or isinstance(ref, dict):
                    if key in ref:
                        ref = ref[key]
                    else:
                        ref = None

                elif isinstance(ref, object):
                    ref = getattr(ref, key)

        if ref:
            if self.partial:
                ref = str(ref)[self.partial[0]:self.partial[1]]
            elif self.split:
                ref = str(ref).split(self.split)

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
    def pipe(self) -> str:
        return self.root.get('pipe')

    @property
    @abstractmethod
    def result(self) -> bool:
        pass


class Condition(ConditionBase):
    @property
    def lparam(self) -> str:
        return self.root.get('lparam')

    @property
    def lparam_value(self):
        return self.parameter_values[self.lparam]

    @property
    def rparam(self) -> str:
        return self.root.get('rparam')

    @property
    def rparam_value(self):
        if not self.rparam:
            return None

        return self.parameter_values[self.rparam]

    @property
    def value(self) -> str:
        return self.root.get('value')

    @property
    def result(self) -> bool:
        lvalue = self.lparam_value
        rvalue = self.rparam_value or self.value

        if self.operator == 'exist':
            return bool(lvalue) == bool(strtobool(rvalue))

        elif self.operator == 'equal':
            return lvalue == rvalue

        elif self.operator == 'time_equal':
            reversed_rvalue = reverse_time(rvalue)
            return (lvalue == rvalue) or (lvalue == reversed_rvalue)

        elif self.operator == 'contains':
            if not lvalue or not rvalue:
                return False

            return (lvalue in rvalue) == bool(strtobool(self.value))

        elif self.operator == 'time_contains':
            if not lvalue or not rvalue:
                return False

            reversed_rvalue = [reverse_time(time) for time in rvalue]
            return ((lvalue in rvalue) or (lvalue in reversed_rvalue)) == bool(strtobool(self.value))

        elif self.operator == 'more':  # 이상
            return float(lvalue) >= float(rvalue)

        elif self.operator == 'below':  # 이하
            return float(lvalue) <= float(rvalue)

        elif self.operator == 'excess':  # 초과
            return float(lvalue) > float(rvalue)

        elif self.operator == 'under':  # 미만
            return float(lvalue) < float(rvalue)

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
        pipe = 'and'

        for condition in self.conditions:
            partial_result = condition.result

            if pipe == 'and':
                result = result and partial_result
            elif pipe == 'or':
                result = result or partial_result

            pipe = condition.pipe

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

    def set_parameter_content(self, content: dict):
        for parameter_name, parameter in self.parameters.items():
            parameter.set_content(content)

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
