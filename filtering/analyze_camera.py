import numpy as np
totals = np.array([
    561230542,
    61934089,
    51798723,
    59600876,
    60035010,
    50422613,
    60622146,
    49599971,
    59697711,
    63242989,
    49915769,
    54916909,
    47422054,
    62510516,
    52720533,
    61796503,
    51547593,
    56735945,
    61810252,
    60647937,
])

capture = np.array([
    12686791,
    9739816,
    10652125,
    10600625,
    12391118,
    12954754,
    12570496,
    10470080,
    14013857,
    11718104,
    14585994,
    9377642,
    15165339,
    14729289,
    15160506,
    9816151,
    14219570,
    15445177,
    15989646,
])

capture_and_serialize = np.array([
    16871664,
    14423741,
    14418949,
    14213737,
    15999772,
    16935832,
    17235004,
    14188403,
    18153272,
    15307508,
    18557113,
    13060673,
    19221001,
    19369546,
    19333671,
    13737478,
    17893434,
    19676427,
    20174728,
])


image_age_at_fiducial = np.array([
    83517266,
    75760190,
    73357307,
    72308660,
    80204781,
    71171595,
    70692876,
])


def main():
    seconds_per_nano = 1e-9
    print("\n\ncapture: ")
    print(capture * seconds_per_nano)
    print("\n\ncapture_and_serialize: ")
    print(capture_and_serialize * seconds_per_nano)
    print("\n\ntotals: ")
    print(totals * seconds_per_nano)
    print("\n\nimage_age_at_fiducial: ")
    print(image_age_at_fiducial * seconds_per_nano)

if __name__ == '__main__':
    main()
