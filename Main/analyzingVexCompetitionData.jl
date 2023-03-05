using Polynomials

# time_periods = [1667019600.0, 1667019600.0, 1667019600.0, 1667624400.0, 1667624400.0, 1667624400.0, 1668060000.0, 1668060000.0, 1668837600.0, 1668837600.0, 1668837600.0, 1668924000.0, 1668924000.0, 1668924000.0, 1670479200.0, 1670479200.0, 1670479200.0]
# scores = [44, 24, 14, 134, 67, 39, 59, 58, 118, 109, 76, 106, 105, 102, 151, 143, 84]

# time_periods_driver = [1667019600.0, 1667019600.0, 1667019600.0, 1667624400.0, 1667624400.0, 1667624400.0, 1668060000.0, 1668060000.0, 1668837600.0, 1668837600.0, 1668837600.0, 1668924000.0, 1668924000.0, 1668924000.0, 1670479200.0, 1670479200.0, 1670479200.0]
# scores_driver =  [44, 24, 14, 115, 61, 39, 56, 54, 100, 88, 76, 100, 91, 82, 100, 96, 67]
# time_periods_program = [1667019600.0, 1667624400.0, 1667624400.0, 1667624400.0, 1668060000.0, 1668060000.0, 1668837600.0, 1668837600.0, 1668837600.0, 1668924000.0, 1668924000.0, 1668924000.0, 1670479200.0, 1670479200.0, 1670479200.0]
# scores_program =  [0, 30, 8, 7, 4, 3, 30, 16, 13, 43, 27, 26, 55, 43, 26]

# indiana
# time_periods_driver = [1667019600.0, 1667019600.0, 1667019600.0, 1667624400.0, 1667624400.0, 1667624400.0, 1668232800.0, 1668232800.0, 1668232800.0, 1668751200.0, 1668751200.0, 1668751200.0, 1668837600.0, 1668837600.0, 1668837600.0, 1669788000.0, 1669788000.0, 1669788000.0, 1669874400.0, 1670047200.0, 1670047200.0, 1670047200.0]
# scores_driver =  [138, 115, 67, 204, 128, 92, 141, 102, 97, 186, 175, 162, 211, 129, 116, 148, 118, 87, 26, 153, 152, 124]
# time_periods_programming = [1667019600.0, 1667019600.0, 1667019600.0, 1667624400.0, 1667624400.0, 1667624400.0, 1668232800.0, 1668232800.0, 1668232800.0, 1668751200.0, 1668751200.0, 1668751200.0, 1668837600.0, 1668837600.0, 1668837600.0, 1669788000.0, 1669788000.0, 1669788000.0, 1669874400.0, 1670047200.0, 1670047200.0, 1670047200.0]
# scores_programming =  [26, 6, 0, 161, 60, 58, 110, 39, 38, 150, 148, 120, 205, 75, 57, 63, 54, 37, 0, 89, 79, 62]
# time_periods_driver = [1658966400.0, 1660176000.0, 1660176000.0, 1660176000.0, 1661990400.0, 1661990400.0, 1661990400.0, 1662595200.0, 1662595200.0, 1662595200.0, 1663200000.0, 1663200000.0, 1663804800.0, 1663804800.0, 1663804800.0, 1664409600.0, 1664409600.0, 1664409600.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665619200.0, 1665619200.0, 1665619200.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666828800.0, 1666828800.0, 1666828800.0, 1667433600.0, 1667433600.0, 1667433600.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668643200.0, 1668643200.0, 1668643200.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669852800.0, 1669852800.0, 1669852800.0, 1670457600.0, 1670457600.0, 1670457600.0]
# scores_driver =  [90, 176, 169, 156, 76, 64, 20, 230, 110, 103, 53, 13, 84, 83, 72, 136, 72, 71, 124, 118, 114, 213, 175, 146, 198, 181, 173, 246, 166, 162, 204, 197, 163, 261, 207, 175, 236, 211, 209, 162, 148, 130, 210, 207, 201, 131, 115, 108]
# time_periods_programming = [1658966400.0, 1660176000.0, 1660176000.0, 1660176000.0, 1661990400.0, 1661990400.0, 1662595200.0, 1662595200.0, 1662595200.0, 1663200000.0, 1663804800.0, 1663804800.0, 1663804800.0, 1664409600.0, 1664409600.0, 1664409600.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665619200.0, 1665619200.0, 1665619200.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666828800.0, 1666828800.0, 1666828800.0, 1667433600.0, 1667433600.0, 1667433600.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668643200.0, 1668643200.0, 1668643200.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669852800.0, 1669852800.0, 1669852800.0, 1670457600.0, 1670457600.0, 1670457600.0]
# scores_programming =  [16, 147, 93, 79, 12, 0, 153, 55, 33, 0, 15, 13, 11, 85, 24, 13, 50, 47, 46, 132, 108, 64, 139, 137, 134, 203, 134, 114, 161, 136, 102, 230, 211, 184, 205, 197, 180, 77, 63, 54, 200, 160, 152, 76, 68, 55]# Path: analyzingVexCompetitionData.jl
# time_periods_driver = [1658966400.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1661990400.0, 1661990400.0, 1661990400.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1663200000.0, 1663200000.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0]
# scores_driver =  [90, 176, 169, 156, 145, 140, 135, 123, 122, 121, 120, 76, 64, 20, 230, 110, 103, 79, 76, 71, 38, 15, 12, 9, 53, 13, 84, 83, 72, 70, 56, 55, 54, 53, 52, 38, 136, 72, 71, 52, 22, 18, 10, 124, 118, 114, 97, 86, 83, 80, 75, 74, 69, 213, 175, 146, 141, 139, 128, 125, 105, 95, 93, 198, 181, 173, 164, 156, 152, 150, 144, 143, 141, 246, 166, 162, 152, 144, 140, 138, 133, 132, 125, 204, 197, 163, 156, 137, 134, 131, 128, 126, 125, 261, 207, 175, 164, 162, 156, 155, 154, 150, 147, 236, 211, 209, 186, 175, 173, 169, 166, 164, 162, 162, 148, 130, 124, 122, 118, 116, 114, 104, 99, 210, 207, 201, 199, 197, 195, 194, 187, 184, 181, 131, 115, 108, 107, 106, 100, 96, 94, 91, 77]
# time_periods_programming = [1658966400.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1661990400.0, 1661990400.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1663200000.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0]
# scores_programming =  [16, 147, 93, 79, 68, 64, 63, 48, 46, 32, 31, 12, 0, 153, 55, 33, 17, 10, 8, 0, 0, 15, 13, 11, 10, 8, 3, 0, 85, 24, 13, 3, 0, 50, 47, 46, 43, 38, 34, 26, 22, 18, 14, 132, 108, 64, 63, 55, 46, 45, 38, 34, 31, 139, 137, 134, 90, 89, 85, 84, 80, 67, 64, 203, 134, 114, 112, 97, 82, 74, 64, 62, 59, 161, 136, 102, 97, 85, 62, 60, 59, 58, 57, 230, 211, 184, 126, 122, 119, 110, 107, 91, 88, 205, 197, 180, 154, 150, 148, 128, 120, 109, 102, 77, 63, 54, 51, 49, 37, 36, 33, 32, 19, 200, 160, 152, 150, 149, 144, 142, 137, 136, 134, 76, 68, 55, 44, 43, 34, 33, 30, 26, 24]
# time_periods_driver = [1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0]
# scores_driver =  [99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 316, 316, 316, 316, 316, 316, 316, 316, 316, 316, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 383, 383, 383, 383, 383, 383, 383, 383, 383, 383, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 219, 219, 219, 219, 219, 219, 219, 219, 219, 219, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 301, 301, 301, 301, 301, 301, 301, 301, 301, 301, 337, 337, 337, 337, 337, 337, 337, 337, 337, 337, 436, 436, 436, 436, 436, 436, 436, 436, 436, 436, 365, 365, 365, 365, 365, 365, 365, 365, 365, 365, 459, 459, 459, 459, 459, 459, 459, 459, 459, 459, 416, 416, 416, 416, 416, 416, 416, 416, 416, 416, 206, 206, 206, 206, 206, 206, 206, 206, 206, 206, 401, 401, 401, 401, 401, 401, 401, 401, 401, 401, 184, 184, 184, 184, 184, 184, 184, 184, 184, 184]
# time_periods_programming = [1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0]
# scores_programming =  [99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 316, 316, 316, 316, 316, 316, 316, 316, 316, 316, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 383, 383, 383, 383, 383, 383, 383, 383, 383, 383, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 219, 219, 219, 219, 219, 219, 219, 219, 219, 219, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 301, 301, 301, 301, 301, 301, 301, 301, 301, 301, 337, 337, 337, 337, 337, 337, 337, 337, 337, 337, 436, 436, 436, 436, 436, 436, 436, 436, 436, 436, 365, 365, 365, 365, 365, 365, 365, 365, 365, 365, 459, 459, 459, 459, 459, 459, 459, 459, 459, 459, 416, 416, 416, 416, 416, 416, 416, 416, 416, 416, 206, 206, 206, 206, 206, 206, 206, 206, 206, 206, 401, 401, 401, 401, 401, 401, 401, 401, 401, 401, 184, 184, 184, 184, 184, 184, 184, 184, 184, 184]



# DEC 26 ---
# time_periods_driver = [1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671667200.0, 1671667200.0, 1671667200.0, 1671667200.0, 1671667200.0, 1671667200.0, 1671667200.0, 1671667200.0, 1671667200.0, 1671667200.0]
# scores_driver =  [110, 110, 110, 110, 110, 110, 110, 110, 110, 110, 316, 316, 316, 316, 316, 316, 316, 316, 316, 316, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 383, 383, 383, 383, 383, 383, 383, 383, 383, 383, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 323, 323, 323, 323, 323, 323, 323, 323, 323, 323, 337, 337, 337, 337, 337, 337, 337, 337, 337, 337, 436, 436, 436, 436, 436, 436, 436, 436, 436, 436, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 459, 459, 459, 459, 459, 459, 459, 459, 459, 459, 416, 416, 416, 416, 416, 416, 416, 416, 416, 416, 206, 206, 206, 206, 206, 206, 206, 206, 206, 206, 401, 401, 401, 401, 401, 401, 401, 401, 401, 401, 456, 456, 456, 456, 456, 456, 456, 456, 456, 456, 464, 464, 464, 464, 464, 464, 464, 464, 464, 464, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158]
# time_periods_programming = [1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1658966400.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1660176000.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1661990400.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1662595200.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663200000.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1663804800.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1664409600.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665014400.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1665619200.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666224000.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1666828800.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1667433600.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668038400.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1668643200.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669248000.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1669852800.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1670457600.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671062400.0, 1671667200.0, 1671667200.0, 1671667200.0, 1671667200.0, 1671667200.0, 1671667200.0, 1671667200.0, 1671667200.0, 1671667200.0, 1671667200.0]
# scores_programming =  [110, 110, 110, 110, 110, 110, 110, 110, 110, 110, 316, 316, 316, 316, 316, 316, 316, 316, 316, 316, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 383, 383, 383, 383, 383, 383, 383, 383, 383, 383, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 323, 323, 323, 323, 323, 323, 323, 323, 323, 323, 337, 337, 337, 337, 337, 337, 337, 337, 337, 337, 436, 436, 436, 436, 436, 436, 436, 436, 436, 436, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 459, 459, 459, 459, 459, 459, 459, 459, 459, 459, 416, 416, 416, 416, 416, 416, 416, 416, 416, 416, 206, 206, 206, 206, 206, 206, 206, 206, 206, 206, 401, 401, 401, 401, 401, 401, 401, 401, 401, 401, 456, 456, 456, 456, 456, 456, 456, 456, 456, 456, 464, 464, 464, 464, 464, 464, 464, 464, 464, 464, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158]



# time_periods_driver = [1666828800.0, 1666828800.0, 1666828800.0, 1667433600.0, 1667433600.0, 1667433600.0, 1668038400.0, 1668038400.0, 1668643200.0, 1668643200.0, 1668643200.0, 1670457600.0, 1670457600.0, 1670457600.0]
# scores_driver =  [44, 24, 14, 134, 67, 39, 59, 58, 118, 118, 118, 151, 151, 96]
# time_periods_programming = [1666828800.0, 1667433600.0, 1667433600.0, 1667433600.0, 1668038400.0, 1668038400.0, 1668643200.0, 1668643200.0, 1668643200.0, 1670457600.0, 1670457600.0, 1670457600.0]
# scores_programming =  [44, 134, 67, 67, 59, 59, 118, 118, 109, 151, 143, 96]


# time_periods_driver = [1666828800.0, 1666828800.0, 1666828800.0, 1667433600.0, 1667433600.0, 1667433600.0, 1668038400.0, 1668038400.0, 1668643200.0, 1668643200.0, 1668643200.0, 1670457600.0, 1670457600.0, 1670457600.0]
# scores_driver =  [44, 24, 14, 115, 61, 39, 56, 54, 100, 91, 88, 100, 96, 67]
# time_periods_programming = [1666828800.0, 1667433600.0, 1667433600.0, 1667433600.0, 1668038400.0, 1668038400.0, 1668643200.0, 1668643200.0, 1668643200.0, 1670457600.0, 1670457600.0, 1670457600.0]
# scores_programming =  [0, 30, 8, 7, 4, 3, 43, 30, 27, 55, 43, 29]


# time_periods_driver = [1666828800.0, 1666828800.0, 1667433600.0, 1667433600.0, 1667433600.0, 1668643200.0, 1668643200.0, 1668643200.0, 1670457600.0, 1670457600.0, 1670457600.0, 1672876800.0, 1672876800.0, 1672876800.0, 1673481600.0, 1673481600.0, 1673481600.0, 1674086400.0, 1674086400.0, 1674086400.0]
# scores_driver =  [24, 4, 115, 61, 26, 100, 88, 82, 100, 96, 67, 127, 123, 109, 103, 81, 79, 86, 78, 68]
# time_periods_programming = [1666828800.0, 1667433600.0, 1667433600.0, 1667433600.0, 1668643200.0, 1668643200.0, 1668643200.0, 1670457600.0, 1670457600.0, 1670457600.0, 1672876800.0, 1672876800.0, 1672876800.0, 1673481600.0, 1673481600.0, 1673481600.0, 1674086400.0, 1674086400.0, 1674086400.0]
# scores_programming =  [0, 30, 8, 6, 78, 30, 26, 55, 43, 29, 89, 77, 50, 67, 62, 55, 46, 35, 33]



# print shape of vector
# println("Shape of time_periods_driver: ", size(time_periods_driver))
# println("Shape of scores_driver_driver: ", size(scores_driver))
# println("Shape of time_periods_programming: ", size(time_periods_programming))
# println("Shape of scores_driver_programming: ", size(scores_programming))

# driver_f = fit(time_periods_driver, scores_driver, 2)
# programming_f = fit(time_periods_programming, scores_programming, 2)
# f = (driver_f + programming_f) 
# println("For Janurary 21st 2023, the average score of the top 10 will be ", f(1674346246))
# println("For Februrary 4th 2023, the average score of the top 10 will be ", f(1675555846))
# println("For March 10th 2023, the average score of the top 10 will be ", f(1678493446))
# println("For April 25th 2023, the average score of the top 10 will be ", f(1682464246))

# println("For Janurary 21st 2023, the average score of the top 10 will be programming ", driver_f(1674346246))
# println("For Janurary 21st 2023, the average score of the top 10 will be programming ", programming_f(1674346246))


times = [1814400.0, 5443200.0, 4233600.0, 4233600.0, 4233600.0, 3628800.0, 1814400.0, 0.0, 7862400.0, 5443200.0, 4838400.0, 0.0, 4233600.0, 3628800.0, 1814400.0, 3628800.0, 6652800.0, 1814400.0, 3628800.0, 4838400.0, 4838400.0, 3024000.0, 7257600.0, 1209600.0, -604800.0, 6048000.0, 604800.0, 6048000.0, 6048000.0, 1814400.0, 6652800.0, 5443200.0, 1814400.0, 4838400.0, 3628800.0, 4233600.0, 4233600.0, 5443200.0, 7862400.0, 9676800.0, 7257600.0, 7862400.0, 6048000.0, 3628800.0, 3628800.0, 6048000.0, 5443200.0, 1209600.0, 6048000.0, 1814400.0, 7257600.0, 3628800.0, 5443200.0, 9676800.0, 7257600.0, 3628800.0, 2419200.0, 3628800.0, 2419200.0, 0.0, 4233600.0, 6048000.0, 2419200.0, 1814400.0, 6048000.0, 6048000.0, 6048000.0, 6048000.0, 2419200.0, 3628800.0, 3024000.0, 1814400.0, 3024000.0, 2419200.0]
scores = [42, 62, 47, 38, 71, 64, 56, 20, 67, 25, 81, 50, 52, 32, 8, 8, 15, 3, 38, 30, 29, 74, 14, 46, 15, 23, 1, 42, 25, 16, 19, 6, 47, 30, 50, 16, 24, 34, 44, 31, 76, 22, 33, 66, 48, 178, 22, 46, 10, 6, 58, 46, 6, 38, 89, 19, 55, 20, 38, 77, 96, 9, 16, 20, 101, 49, 138, 17, 7, 42, 7, 54, 28, 23]

# times = [1814400.0, 5443200.0, 4233600.0, 4233600.0, 4233600.0, 3628800.0, 1814400.0, 0.0, 7862400.0, 5443200.0, 4838400.0, 0.0, 4233600.0, 3628800.0, 1814400.0, 3628800.0, 6652800.0, 1814400.0, 3628800.0, 4838400.0, 3628800.0, 4838400.0, 3024000.0, 8467200.0, 7257600.0, 1209600.0, -604800.0, 6048000.0, 604800.0, 6048000.0, 6048000.0, 1814400.0, 6652800.0, 1814400.0, 5443200.0, 1814400.0, 4838400.0, 3628800.0, 4233600.0, 4233600.0, 5443200.0, 7862400.0, 9676800.0, 7257600.0, 7862400.0, 1814400.0, 6048000.0, 3628800.0, 3628800.0, 6048000.0, 5443200.0, 1209600.0, 6048000.0, 1814400.0, 7257600.0, 3628800.0, 5443200.0, 9676800.0, 7257600.0, 3628800.0, 2419200.0, 3628800.0, 2419200.0, 0.0, 4233600.0, 6048000.0, 2419200.0, 1814400.0, 6048000.0, 6048000.0, 6048000.0, 6048000.0, 2419200.0, 3628800.0, 3628800.0, 3024000.0, 1814400.0, 1814400.0, 3024000.0, 4233600.0, 2419200.0, 1814400.0]
# scores = [42, 62, 47, 38, 71, 64, 56, 20, 67, 25, 81, 50, 52, 32, 8, 8, 15, 3, 38, 30, 113, 29, 74, 23, 14, 46, 15, 23, 1, 42, 25, 16, 19, 14, 6, 47, 30, 50, 16, 24, 34, 44, 31, 76, 22, 65, 33, 66, 48, 178, 22, 46, 10, 6, 58, 46, 6, 38, 89, 19, 55, 20, 38, 77, 96, 9, 16, 20, 101, 49, 138, 17, 7, 42, 36, 7, 54, 19, 28, 119, 23, 186]
times = [1814400.0, 5443200.0, 4233600.0, 4233600.0, 4233600.0, 3628800.0, 1814400.0, 7862400.0, 5443200.0, 4838400.0, 4233600.0, 3628800.0, 1814400.0, 3628800.0, 6652800.0, 1814400.0, 3628800.0, 4838400.0, 4838400.0, 3024000.0, 7257600.0, 1209600.0, 6048000.0, 604800.0, 6048000.0, 6048000.0, 1814400.0, 6652800.0, 5443200.0, 1814400.0, 4838400.0, 3628800.0, 4233600.0, 4233600.0, 5443200.0, 7862400.0, 9676800.0, 7257600.0, 7862400.0, 6048000.0, 3628800.0, 3628800.0, 6048000.0, 5443200.0, 1209600.0, 6048000.0, 1814400.0, 7257600.0, 3628800.0, 5443200.0, 9676800.0, 7257600.0, 3628800.0, 2419200.0, 3628800.0, 2419200.0, 4233600.0, 6048000.0, 2419200.0, 1814400.0, 6048000.0, 6048000.0, 6048000.0, 6048000.0, 2419200.0, 3628800.0, 3024000.0, 1814400.0, 3024000.0, 2419200.0]
scores = [42, 62, 47, 38, 71, 64, 56, 67, 25, 81, 52, 32, 8, 8, 15, 3, 38, 30, 29, 74, 14, 46, 23, 1, 42, 25, 16, 19, 6, 47, 30, 50, 16, 24, 34, 44, 31, 76, 22, 33, 66, 48, 178, 22, 46, 10, 6, 58, 46, 6, 38, 89, 19, 55, 20, 38, 96, 9, 16, 20, 101, 49, 138, 17, 7, 42, 7, 54, 28, 23]

times = [1814400.0, 5443200.0, 4233600.0, 4233600.0, 4233600.0, 3628800.0, 1814400.0, 7862400.0, 5443200.0, 4838400.0, 4233600.0, 3628800.0, 1814400.0, 3628800.0, 6652800.0, 1814400.0, 3628800.0, 4838400.0, 4838400.0, 3024000.0, 7257600.0, 1209600.0, 6048000.0, 604800.0, 6048000.0, 6048000.0, 1814400.0, 6652800.0, 5443200.0, 1814400.0, 4838400.0, 3628800.0, 4233600.0, 4233600.0, 5443200.0, 7862400.0, 9676800.0, 7257600.0, 7862400.0, 6048000.0, 3628800.0, 3628800.0, 6048000.0, 5443200.0, 1209600.0, 6048000.0, 1814400.0, 7257600.0, 3628800.0, 5443200.0, 9676800.0, 7257600.0, 3628800.0, 2419200.0, 3628800.0, 2419200.0, 4233600.0, 6048000.0, 2419200.0, 1814400.0, 6048000.0, 6048000.0, 6048000.0, 6048000.0, 2419200.0, 3628800.0, 3024000.0, 1814400.0, 3024000.0, 2419200.0]
scores = [42, 62, 47, 38, 71, 64, 56, 67, 25, 81, 52, 32, 8, 8, 15, 3, 38, 30, 29, 74, 14, 46, 23, 1, 42, 25, 16, 19, 6, 47, 30, 50, 16, 24, 34, 44, 31, 76, 22, 33, 66, 48, 178, 22, 46, 10, 6, 58, 46, 6, 38, 89, 19, 55, 20, 38, 96, 9, 16, 20, 101, 49, 138, 17, 7, 42, 7, 54, 28, 23]

times = [1814400.0, 5443200.0, 4233600.0, 4233600.0, 4233600.0, 4233600.0, 3628800.0, 1814400.0, 8467200.0, 7862400.0, 5443200.0, 8467200.0, 4838400.0, 3628800.0, 4233600.0, 3628800.0, 604800.0, 1814400.0, 3628800.0, 8467200.0, 8467200.0, 6652800.0, 1814400.0, 8467200.0, 1814400.0, 3628800.0, 4838400.0, 3628800.0, 4838400.0, 3024000.0, 8467200.0, 7257600.0, 1209600.0, 1814400.0, 6048000.0, 604800.0, 6048000.0, 6048000.0, 1814400.0, 1814400.0, 6652800.0, 1814400.0, 5443200.0, 1814400.0, 1209600.0, 4838400.0, 3628800.0, 4233600.0, 4233600.0, 5443200.0, 4233600.0, 7862400.0, 1814400.0, 9676800.0, 604800.0, 7257600.0, 3024000.0, 7862400.0, 1814400.0, 6048000.0, 3628800.0, 3628800.0, 6048000.0, 5443200.0, 4233600.0, 1209600.0, 6048000.0, 1814400.0, 1209600.0, 7257600.0, 3024000.0, 3628800.0, 5443200.0, 1209600.0, 8467200.0, 9676800.0, 7257600.0, 3024000.0, 4233600.0, 6048000.0, 2419200.0, 1814400.0, 6048000.0, 6048000.0, 6048000.0, 3628800.0, 2419200.0, 6048000.0, 3628800.0, 2419200.0, 2419200.0, 3628800.0, 3628800.0, 3628800.0, 3024000.0, 604800.0, 1814400.0, 1814400.0, 3024000.0, 604800.0, 4233600.0, 3628800.0, 3628800.0, 2419200.0, 1814400.0, 4233600.0, 1814400.0, 1814400.0, 604800.0, 604800.0, 1814400.0, 1814400.0, 604800.0, 604800.0, 604800.0]
scores = [42, 62, 47, 38, 58, 71, 64, 56, 24, 67, 25, 71, 81, 30, 52, 32, 57, 8, 8, 11, 44, 15, 21, 41, 3, 38, 30, 113, 29, 74, 23, 14, 46, 106, 23, 1, 42, 25, 16, 44, 19, 14, 6, 47, 43, 30, 50, 16, 24, 34, 27, 44, 59, 31, 26, 76, 143, 22, 65, 33, 66, 48, 178, 22, 37, 46, 10, 6, 88, 58, 50, 46, 6, 35, 66, 38, 89, 5, 96, 9, 16, 20, 101, 49, 138, 19, 55, 17, 20, 38, 7, 42, 32, 36, 7, 83, 54, 19, 28, 46, 119, 16, 6, 23, 6, 43, 81, 14, 47, 35, 186, 63, 72, 58, 56]

f = (fit(times, scores, 2))
