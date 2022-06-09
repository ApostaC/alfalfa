#include <vector>
#include <cassert>
#include <iostream>
#include <random>
#include "comp_encoder.hh"
#include "timestamp.hh"

using namespace std;

static const int kFecRateTableSize = 6450;

static const unsigned char kFecRateTable[kFecRateTableSize] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,
    11,  11,  11,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
    39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
    39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
    39,  39,  39,  39,  39,  39,  51,  51,  51,  51,  51,  51,  51,  51,  51,
    51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,
    51,  51,  51,  51,  51,  51,  51,  51,  51,  0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   8,   8,   8,
    8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   30,  30,  30,
    30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  56,  56,  56,
    56,  56,  56,  56,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,
    65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,
    65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,  65,
    87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,  87,
    87,  87,  87,  87,  87,  87,  87,  87,  87,  78,  78,  78,  78,  78,  78,
    78,  78,  78,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   6,   6,   6,   23,  23,  23,  23,  23,  23,  23,  23,  23,
    23,  23,  23,  23,  23,  23,  44,  44,  44,  44,  44,  44,  50,  50,  50,
    50,  50,  50,  50,  50,  50,  68,  68,  68,  68,  68,  68,  68,  85,  85,
    85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,
    85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,  85,
    85,  85,  85,  85,  85,  85,  85,  85,  85,  105, 105, 105, 105, 105, 105,
    105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105,
    105, 105, 105, 88,  88,  88,  88,  88,  88,  88,  88,  88,  0,   0,   0,
    0,   0,   0,   0,   0,   0,   5,   5,   5,   5,   5,   5,   19,  19,  19,
    36,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,
    55,  55,  55,  55,  55,  55,  69,  69,  69,  69,  69,  69,  69,  69,  69,
    75,  75,  80,  80,  80,  80,  80,  97,  97,  97,  97,  97,  97,  97,  97,
    97,  97,  102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102,
    102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102,
    102, 102, 102, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116,
    116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 116, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 0,   0,   0,   0,   0,   0,   0,   0,   4,
    16,  16,  16,  16,  16,  16,  30,  35,  35,  47,  58,  58,  58,  58,  58,
    58,  58,  58,  58,  58,  58,  58,  58,  58,  63,  63,  63,  63,  63,  63,
    77,  77,  77,  77,  77,  77,  77,  82,  82,  82,  82,  94,  94,  94,  94,
    94,  105, 105, 105, 105, 110, 110, 110, 110, 110, 110, 122, 122, 122, 122,
    122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122,
    122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 115, 115, 115, 115, 115, 115, 115, 115, 115,
    0,   0,   0,   0,   0,   0,   0,   4,   14,  27,  27,  27,  27,  27,  31,
    41,  52,  52,  56,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,
    69,  69,  69,  69,  69,  69,  69,  69,  69,  79,  79,  79,  79,  83,  83,
    83,  94,  94,  94,  94,  106, 106, 106, 106, 106, 115, 115, 115, 115, 125,
    125, 125, 125, 125, 125, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   0,   0,   0,   3,   3,
    3,   17,  28,  38,  38,  38,  38,  38,  47,  51,  63,  63,  63,  72,  72,
    72,  72,  72,  72,  72,  76,  76,  76,  76,  80,  80,  80,  80,  80,  80,
    80,  80,  80,  84,  84,  84,  84,  93,  93,  93,  105, 105, 105, 105, 114,
    114, 114, 114, 114, 124, 124, 124, 124, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 0,   0,   0,   0,   12,  12,  12,  35,  43,  47,  47,  47,
    47,  47,  58,  58,  66,  66,  66,  70,  70,  70,  70,  70,  73,  73,  82,
    82,  82,  86,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,  94,
    94,  105, 105, 105, 114, 114, 114, 114, 117, 117, 117, 117, 117, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   0,   0,
    0,   24,  24,  24,  49,  53,  53,  53,  53,  53,  53,  61,  61,  64,  64,
    64,  64,  70,  70,  70,  70,  78,  78,  88,  88,  88,  96,  106, 106, 106,
    106, 106, 106, 106, 106, 106, 106, 112, 112, 112, 120, 120, 120, 124, 124,
    124, 124, 124, 124, 124, 124, 124, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 0,   0,   0,   5,   36,  36,  36,  55,  55,
    55,  55,  55,  55,  55,  58,  58,  58,  58,  58,  64,  78,  78,  78,  78,
    87,  87,  94,  94,  94,  103, 110, 110, 110, 110, 110, 110, 110, 110, 116,
    116, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    0,   0,   0,   18,  43,  43,  43,  53,  53,  53,  53,  53,  53,  53,  53,
    58,  58,  58,  58,  71,  87,  87,  87,  87,  94,  94,  97,  97,  97,  109,
    111, 111, 111, 111, 111, 111, 111, 111, 125, 125, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   0,   0,   31,  46,  46,
    46,  48,  48,  48,  48,  48,  48,  48,  48,  66,  66,  66,  66,  80,  93,
    93,  93,  93,  95,  95,  95,  95,  100, 115, 115, 115, 115, 115, 115, 115,
    115, 115, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 0,   0,   4,   40,  45,  45,  45,  45,  45,  45,  45,  45,
    49,  49,  49,  74,  74,  74,  74,  86,  90,  90,  90,  90,  95,  95,  95,
    95,  106, 120, 120, 120, 120, 120, 120, 120, 120, 120, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   0,   14,
    42,  42,  42,  42,  42,  42,  42,  42,  46,  56,  56,  56,  80,  80,  80,
    80,  84,  84,  84,  84,  88,  99,  99,  99,  99,  111, 122, 122, 122, 122,
    122, 122, 122, 122, 122, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 0,   0,   26,  40,  40,  40,  40,  40,  40,
    40,  40,  54,  66,  66,  66,  80,  80,  80,  80,  80,  80,  80,  84,  94,
    106, 106, 106, 106, 116, 120, 120, 120, 120, 120, 120, 120, 120, 124, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    0,   3,   34,  38,  38,  38,  38,  38,  42,  42,  42,  63,  72,  72,  76,
    80,  80,  80,  80,  80,  80,  80,  89,  101, 114, 114, 114, 114, 118, 118,
    118, 118, 118, 118, 118, 118, 118, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   12,  36,  36,  36,  36,
    36,  36,  49,  49,  49,  69,  73,  76,  86,  86,  86,  86,  86,  86,  86,
    86,  97,  109, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122, 122,
    122, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 0,   22,  34,  34,  34,  34,  38,  38,  57,  57,  57,  69,
    73,  82,  92,  92,  92,  92,  92,  92,  96,  96,  104, 117, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   29,  33,
    33,  33,  33,  44,  44,  62,  62,  62,  69,  77,  87,  95,  95,  95,  95,
    95,  95,  107, 107, 110, 120, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 0,   31,  31,  31,  31,  31,  51,  51,  62,
    65,  65,  73,  83,  91,  94,  94,  94,  94,  97,  97,  114, 114, 114, 122,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    0,   29,  29,  29,  29,  29,  56,  56,  59,  70,  70,  79,  86,  89,  89,
    89,  89,  89,  100, 100, 116, 116, 116, 122, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   28,  28,  28,  28,  28,
    57,  57,  57,  76,  76,  83,  86,  86,  86,  86,  86,  89,  104, 104, 114,
    114, 114, 124, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 0,   27,  27,  27,  27,  30,  55,  55,  55,  80,  80,  83,
    86,  86,  86,  86,  86,  93,  108, 108, 111, 111, 111, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   26,  26,
    26,  26,  36,  53,  53,  53,  80,  80,  80,  90,  90,  90,  90,  90,  98,
    107, 107, 107, 107, 107, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 0,   26,  26,  26,  28,  42,  52,  54,  54,
    78,  78,  78,  95,  95,  95,  97,  97,  104, 106, 106, 106, 106, 106, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    0,   24,  24,  24,  33,  47,  49,  58,  58,  74,  74,  74,  97,  97,  97,
    106, 106, 108, 108, 108, 108, 108, 108, 124, 124, 124, 124, 124, 124, 124,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   24,  24,  24,  39,  48,
    50,  63,  63,  72,  74,  74,  96,  96,  96,  109, 111, 111, 111, 111, 111,
    111, 111, 119, 119, 122, 122, 122, 122, 122, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 0,   23,  23,  23,  43,  46,  54,  66,  66,  69,  77,  77,
    92,  92,  92,  105, 113, 113, 113, 113, 113, 113, 113, 115, 117, 123, 123,
    123, 123, 123, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   22,  22,
    22,  44,  44,  59,  67,  67,  67,  81,  81,  89,  89,  89,  97,  112, 112,
    112, 112, 112, 112, 112, 112, 119, 126, 126, 126, 126, 126, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 0,   21,  21,  24,  43,  45,  63,  65,  65,
    67,  85,  85,  87,  87,  87,  91,  109, 109, 109, 111, 111, 111, 111, 111,
    123, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    0,   21,  21,  28,  42,  50,  63,  63,  66,  71,  85,  85,  85,  85,  87,
    92,  106, 106, 108, 114, 114, 114, 114, 114, 125, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   20,  20,  34,  41,  54,
    62,  62,  69,  75,  82,  82,  82,  82,  92,  98,  105, 105, 110, 117, 117,
    117, 117, 117, 124, 124, 126, 126, 126, 126, 126, 126, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 0,   20,  20,  38,  40,  58,  60,  60,  73,  78,  80,  80,
    80,  80,  100, 105, 107, 107, 113, 118, 118, 118, 118, 118, 120, 120, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   19,  21,
    38,  40,  58,  58,  60,  75,  77,  77,  77,  81,  81,  107, 109, 109, 109,
    114, 116, 116, 116, 116, 116, 116, 116, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 0,   18,  25,  37,  44,  56,  56,  63,  75,
    75,  75,  75,  88,  88,  111, 111, 111, 111, 112, 112, 112, 112, 112, 112,
    112, 114, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    0,   18,  30,  36,  48,  55,  55,  67,  73,  73,  73,  73,  97,  97,  110,
    110, 110, 110, 110, 110, 110, 110, 110, 110, 110, 116, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   18,  34,  36,  52,  55,
    55,  70,  72,  73,  73,  73,  102, 104, 108, 108, 108, 108, 109, 109, 109,
    109, 109, 109, 109, 119, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 0,   17,  35,  35,  52,  59,  59,  70,  70,  76,  76,  76,
    99,  105, 105, 105, 105, 105, 111, 111, 111, 111, 111, 111, 111, 121, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   17,  34,
    36,  51,  61,  62,  70,  70,  80,  80,  80,  93,  103, 103, 103, 103, 103,
    112, 112, 112, 112, 112, 116, 118, 124, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 0,   16,  33,  39,  50,  59,  65,  72,  72,
    82,  82,  82,  91,  100, 100, 100, 100, 100, 109, 109, 109, 109, 109, 121,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    0,   16,  32,  43,  48,  54,  66,  75,  75,  81,  83,  83,  92,  97,  97,
    97,  99,  99,  105, 105, 105, 105, 105, 123, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   15,  31,  46,  47,  49,
    69,  77,  77,  81,  85,  85,  93,  95,  95,  95,  100, 100, 102, 102, 102,
    102, 102, 120, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 0,   15,  30,  46,  48,  48,  70,  75,  79,  82,  87,  87,
    92,  94,  94,  94,  103, 103, 103, 103, 103, 104, 104, 115, 120, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   15,  30,
    45,  50,  50,  68,  70,  80,  85,  89,  89,  90,  95,  95,  95,  104, 104,
    104, 104, 104, 109, 109, 112, 114, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 0,   14,  29,  44,  54,  54,  64,  64,  83,
    87,  88,  88,  88,  98,  98,  98,  103, 103, 103, 103, 103, 113, 113, 113,
    113, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    0,   14,  29,  43,  56,  56,  61,  61,  84,  85,  88,  88,  88,  100, 100,
    100, 102, 102, 102, 102, 102, 113, 116, 116, 116, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   14,  28,  42,  57,  57,
    62,  62,  80,  80,  91,  91,  91,  100, 100, 100, 100, 100, 100, 100, 100,
    109, 119, 119, 119, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 0,   14,  28,  42,  56,  56,  65,  66,  76,  76,  92,  92,
    92,  97,  97,  97,  101, 101, 101, 101, 101, 106, 121, 121, 121, 126, 126,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 0,   13,  27,
    41,  55,  55,  67,  72,  74,  74,  90,  90,  90,  91,  91,  91,  105, 105,
    105, 105, 105, 107, 122, 122, 122, 123, 123, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 0,   13,  27,  40,  54,  54,  67,  76,  76,
    76,  85,  85,  85,  85,  85,  85,  112, 112, 112, 112, 112, 112, 121, 121,
    121, 121, 121, 126, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
};

BasicEncoder::BasicEncoder(uint32_t init_bitrate_byteps, uint16_t fps)
  : target_bitrate_byteps_(init_bitrate_byteps), fps_(fps), stats_recoder_("/tmp/encoder.csv")
{
  stats_recoder_.post_updates(0, 0, 0);
}

BasicEncoder::BasicEncoder(uint32_t init_bitrate_byteps, uint16_t fps, const string & stats_file)
  : target_bitrate_byteps_(init_bitrate_byteps), fps_(fps), stats_recoder_(stats_file)
{
}

void BasicEncoder::set_loss_rate(double loss_rate)
{
  /* compute FEC rate using webrtc logic */
  {
    //int bitrate_kbps = max(target_bitrate_byteps_ / 125, 200u);
    //int rate_lvl = (bitrate_kbps - 200) / 100;
    //rate_lvl = min(rate_lvl, 49);
    //int loss_lvl = loss_rate * 256;
    //loss_lvl = min(loss_lvl, 128);

    ////cerr << "rate lvl = " << rate_lvl << endl;

    //uint8_t code_rate = kFecRateTable[rate_lvl * 129 + loss_lvl];
    //fec_rate_ = code_rate * 2;
  }

  /* computing FEC rate using pre-configured protection overhead */
  auto fec_ratio = min(loss_rate * protection_overhead_, 0.5);
  auto fec_rate_d = min(fec_ratio / (1 - fec_ratio), 1.);
  fec_rate_ = 255 * fec_rate_d;
}

Optional<FragmentedFrame> BasicEncoder::encode_next_frame(uint32_t curr_timestamp_ms)
{
  frame_id_ += 1;
  std::random_device rd;
  std::ranlux24 e(rd());
  std::normal_distribution<double> dis(1, 1e-5);
  auto multiplier = dis(e);
  uint32_t base_size_bytes = target_bitrate_byteps_ * 1 / fps_;

  /* consider fec */
  base_size_bytes = base_size_bytes / (1 + (fec_rate_ / 255.0f));

  uint32_t real_bytes = std::floor(multiplier * base_size_bytes);
  if (real_bytes == 0) {
    //return {};
    real_bytes = 10; // generate a very small frame
  }
  std::vector<uint8_t> data;
  data.resize(real_bytes);

  // note: time_to_next_frame is in MICRO-SECONDS (us)
  bool is_key_frame = (frame_id_ % gop_ == 1);
  FragmentedFrame ret(0, 0, 0, frame_id_, 1000000 / fps_, data, is_key_frame, fec_rate_);

  // update frame_observer
  for (auto obs : frame_observers_) {
    obs->new_complete_frame(curr_timestamp_ms, ret);
  }
  cerr << "Encoding a new frame: " << frame_id_ << ", size = " << real_bytes 
       << " tgt_br = " << target_bitrate_byteps_ << ", fec_rate = " << fec_rate_ / 255.0 << endl;
  return ret;
}

std::vector<Packet> BasicEncoder::encode_next_frame_packets(uint32_t curr_timestamp_ms)
{
  stats_recoder_.post_updates(target_bitrate_byteps_, target_bitrate_byteps_, fec_rate_ / (fec_rate_ + 255.0));
  auto frame = encode_next_frame(curr_timestamp_ms);
  if (frame.initialized()) {
    return frame.get().packets();
  }
  return {};
}

/* SVC Codec */
SVCEncoder::SVCEncoder(uint32_t init_bitrate, uint16_t fps,
                       const vector<double> & size_ratios, const vector<uint8_t> & fec_rates,
                       const string & stats_file)
  : size_ratios_(size_ratios), fec_rates_(fec_rates), stats_recoder_(stats_file)
{
  /* initialize layer_encoders_ */
  assert(size_ratios.size() == fec_rates.size());
  for (unsigned i = 0; i < size_ratios.size(); i++) {
    uint32_t init_rate = init_bitrate * size_ratios[i];
    auto fec_rate = fec_rates[i];
    layer_encoders_.emplace_back(init_rate, fps);

    /* set fec rate */
    layer_encoders_.back().set_fec_rate(fec_rate);
    layer_encoders_.back().set_protection_overhead(1);
  }
}

SVCEncoder::SVCEncoder(uint32_t init_bitrate, uint16_t fps,
                       const vector<double> & size_ratios, const vector<uint8_t> & fec_rates)
  : SVCEncoder(init_bitrate, fps, size_ratios, fec_rates, "/tmp/svc_encoder.csv")
{}

void SVCEncoder::set_target_bitrate(uint32_t bitrate_byteps) 
{
  for (unsigned i = 0; i < size_ratios_.size(); i++) {
    uint32_t rate_byteps = bitrate_byteps * size_ratios_[i];
    layer_encoders_[i].set_target_bitrate(rate_byteps);
  }
}

void SVCEncoder::set_gop(uint32_t gop)
{
  gop_ = gop;
  for (auto & encoder : layer_encoders_) {
    encoder.set_gop(gop);
  }
}

Optional<FragmentedFrame> SVCEncoder::encode_next_frame(uint32_t)
{
  throw runtime_error("Please call SVCEncoder::encode_next_frame_packets instead of SVCEncoder::encode_next_frame");
}

std::vector<Packet> SVCEncoder::encode_next_frame_packets(uint32_t curr_timestamp_ms)
{
  frame_id_ += 1;

  /* encode layers */
  std::vector<FragmentedFrame> temp_frames;
  uint32_t layer_id = 0;
  for (auto & encoder : layer_encoders_) {
    auto frame_handle = encoder.encode_next_frame(curr_timestamp_ms);
    if (not frame_handle.initialized()) {
      throw runtime_error("SVCEncoder::encode_next_frame_packets: failed to encode a new frame!");
    }

    temp_frames.emplace_back(move(frame_handle.get()));
    temp_frames.back().set_frame_no(layer_id);
    layer_id++;
  }

  /* combine to get a new svc frame */
  bool is_key_frame = (frame_id_ % gop_ == 1);
  SVCFrame svc_frame(frame_id_, move(temp_frames));
  if (is_key_frame) svc_frame.set_key_frame();

  for (auto obs : frame_observers_) {
    obs->new_complete_frame(curr_timestamp_ms, svc_frame.to_fragmented_frame().get());
  }
  return svc_frame.fragments();
}
