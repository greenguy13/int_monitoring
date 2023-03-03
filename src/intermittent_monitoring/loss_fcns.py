#!/usr/bin/env python

import math

#Perhaps we remove these global variables and insert them as parameters to the functions instead
# global max_fmeasure
# global fsafe
# max_fmeasure = 100
# fsafe = 80

def decay(rate, t, starting_value):
    """
    Decay function
    :param rate:
    :param t:
    :return:
    """
    decayed_fmeasure = starting_value*(1.0 - rate)**t
    return decayed_fmeasure

def get_time_given_decay(max_fmeasure, decayed_fmeasure, rate):
    """
    Retrieves time given decayed measure and decay rate (by inversion)
    :param decayed_measure:
    :param rate:
    :return:
    """
    t = math.log((decayed_fmeasure/max_fmeasure), 1.0-rate)

    return t

def beta_rate(rate, rates):
    """
    Computes the normalized F-measure decay rate, which then is the Loss growth rate
    :param rate (float): decay rate
    :param rates (list): list of decay rates
    :return:
    """
    beta = rate / sum(rates)
    return beta

# def compute_loss(max_fmeasure, decayed_fmeasure, fsafe, rate, rates, est_duration):
#     """
#     Computes loss by estimating the decayed fmeasure given the decay rate after a set duration
#     :param decayed_fmeasure:
#     :param rate:
#     :param duration:
#     :return:
#     """
#     t0 = get_time_given_decay(max_fmeasure, decayed_fmeasure, rate)
#     t = t0 + est_duration
#     est_decayed_fmeasure = decay(rate, t, max_fmeasure)
#     print("Current measure: {}. Estimated decayed measure: {}".format(decayed_fmeasure, est_decayed_fmeasure))
#
#     # This part here needs fixing
#     if est_decayed_fmeasure < fsafe:
#         diff = fsafe - est_decayed_fmeasure
#     else:
#         diff = 0
#
#     beta = beta_rate(rate, rates)
#
#     loss = beta*diff**2
#
#     return float(loss)

# def compute_cost_fmeasures(fmeasures, decay_rates, fsafe):
#     """
#     Computes the cost, (i.e., the sum of losses), given the F-measures
#     :param fmeasures: dict of F-measures of areas
#     :param decay_rates: dict of decay rates of areas
#     :return:
#     """
#     cost = 0
#     areas = fmeasures.keys()
#     for area in areas:
#         if fmeasures[area] < fsafe:
#             diff = fsafe - fmeasures[area]
#         else:
#             diff = 0
#
#         beta = beta_rate(decay_rates[area], list(decay_rates.values()))
#         loss = beta*diff**2
#         cost += loss
#     return cost

def compute_loss(max_fmeasure, decayed_fmeasure, fsafe, fcrit, rate, est_duration):
    """
    Computes loss by estimating the decayed fmeasure given the decay rate after a set duration
    :param decayed_fmeasure:
    :param rate:
    :param duration:
    :return:
    """
    t0 = get_time_given_decay(max_fmeasure, decayed_fmeasure, rate)
    t = t0 + est_duration
    est_decayed_fmeasure = decay(rate, t, max_fmeasure)
    print("Current measure: {}. Estimated decayed measure: {}".format(decayed_fmeasure, est_decayed_fmeasure))

    # Safe zone
    if est_decayed_fmeasure >= fsafe:
        loss = fsafe - est_decayed_fmeasure #0
    # Caution zone
    elif fcrit <= est_decayed_fmeasure and est_decayed_fmeasure < fsafe:
        loss = 2*(fsafe - est_decayed_fmeasure) #fsafe - est_decayed_fmeasure
    # Crit zone
    elif est_decayed_fmeasure < fcrit:
        loss = (fsafe - est_decayed_fmeasure)**2

    return float(loss)

def compute_cost_fmeasures(fmeasures, fsafe, fcrit):
    """
    Computes the cost, (i.e., the sum of losses), given the F-measures
    :param fmeasures: dict of F-measures of areas
    :return:
    """
    cost = 0
    areas = fmeasures.keys()
    for area in areas:
        if fmeasures[area] >= fsafe:
            loss = fsafe - fmeasures[area] #0
        elif fcrit <= fmeasures[area] and fmeasures[area] < fsafe:
            loss = 2*(fsafe - fmeasures[area]) #fsafe - fmeasures[area]
        elif fmeasures[area] < fcrit:
            loss = (fsafe - fmeasures[area])**2
        cost += loss
    return cost

