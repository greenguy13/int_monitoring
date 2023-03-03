#!/usr/bin/env python
"""
Pruning strategy for tree-based decision making
"""


def is_feasible(battery, battery_consumption):
    """
    If current battery level can cover battery consumption to an area then back to the charging station.
    :param battery_level: current battery level
    :param battery_consumption: battery consumed to monitor an area then back to charging station
    :return:
    """
    #NOTE: WE HAVE TO MAKE SURE THAT THERE IS ENOUGH BATTERY TO GO BACK TO THE CHARGING STATION
    if battery >= battery_consumption:
        return True
    else:
        return False

def prune(battery, battery_consumption, decayed_fmeasure, safe_fmeasure):
    """
    Prunes a branch growing from a node if it is infeasible or if in the next decision step it is still in safe.
    Equivalently: If feasible and F-measure is below safe, do not prune. Else, prune.

    Note: If decayed_fmeasure is None, then the criteria for pruning is feasibility
    :return:
    """

    if decayed_fmeasure is not None:
        print("Is feasible: {}. Fmeasure at least safe: {}. Prune: {}".format(is_feasible(battery, battery_consumption), decayed_fmeasure >= safe_fmeasure, (is_feasible(battery, battery_consumption) is False) or (decayed_fmeasure >= safe_fmeasure)))
        if (is_feasible(battery, battery_consumption) is False) or (decayed_fmeasure >= safe_fmeasure):
            return True

        else:
            return False

    else:
        print("Is feasible: {}. Prune: {}".format(is_feasible(battery, battery_consumption), is_feasible(battery, battery_consumption) is False))
        if is_feasible(battery, battery_consumption) is False:
            return True
        else:
            return False
