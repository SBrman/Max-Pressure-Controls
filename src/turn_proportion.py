#! python3

__author__ = "Simanta Barman"
__email__ = "barma017@umn.edu"

import json
import xml.etree.ElementTree as ET
from typing import Union


class TurnProportionJSON:
    def __init__(self, path):
        with open(path, 'r') as turn_file:
            self._turn_proportions = json.loads(turn_file.read())

    def turn_proportion(self, upstream: Union[str, int], downstream: Union[str, int]):
        """
        returns the turn proportions given upstream and downstream links
        """
        upstream = str(upstream)
        downstream = str(downstream)
        try:
            return self._turn_proportions[upstream][downstream]
        except KeyError:
            return 0


class TurnProportionXML:

    turn_proportion = TurnProportionJSON.turn_proportion

    def __init__(self, path):
        self.path = path
        self.root = ET.parse(path).getroot()

        while self.root.tag != 'interval':
            self.root = list(self.root)[0]

        self.turn_data = {}
        self._turn_proportions = {}
        self.__read_turn_count()
        if not self._turn_proportions:
            self.__proportions()

    def id(self):
        return self.root.attrib['id']

    def begin(self):
        return self.root.attrib['begin']

    def end(self):
        return self.root.attrib['end']

    def __read_turn_count(self):
        """Reads and saves the turn counts."""
        for er in self.root:
            _from = er.attrib.get('via', er.attrib['from'])
            to = er.attrib['to']
            self._turn_proportions.setdefault(_from, {})
            try:
                count = int(er.attrib['count'])
                self.turn_data[_from][to] = count
            except KeyError:
                self._turn_proportions[_from][to] = float(er.attrib['probability'])

    def __proportions(self):
        """For initiliazing the proportions dict."""
        for _from, to_count in self.turn_data.items():
            self._turn_proportions.setdefault(_from, {})
            for to, count in to_count.items():
                try:
                    self._turn_proportions[_from][to] = count / sum(to_count.values())
                except ZeroDivisionError:
                    self._turn_proportions[_from][to] = 0


if __name__ == "__main__":
##    turnFile = r'F:/MPResearch/MPPROJECT/sumo_models/fixed/i3/turn_count/d1_turnCount_am_off_peak_1.xml'

    tf = r"F:/te/busmp/net/turnRatio.add.xml"
    d1 = TurnProportionXML(tf)

##    i, j = 'CR109_LR_2', 'CR109_LR_3'
##    print(f"Turn proportion from \"{i}\" to \"{j}\" = {d1.turn_proportion(i, j)}")
##
##    turnPath = r"F:\Austin\net\turn_ratios.json"
##    demand = TurnProportionJSON(turnPath)

