#ifndef TOKENS_HH
#define TOKENS_HH

#include "bool_decoder.hh"

enum token {
  ZERO_TOKEN,
  ONE_TOKEN,
  TWO_TOKEN,
  THREE_TOKEN,
  FOUR_TOKEN,
  DCT_VAL_CATEGORY1,
  DCT_VAL_CATEGORY2,
  DCT_VAL_CATEGORY3,
  DCT_VAL_CATEGORY4,
  DCT_VAL_CATEGORY5,
  DCT_VAL_CATEGORY6,
  DCT_EOB_TOKEN
};

const unsigned int ENTROPY_NODES = DCT_VAL_CATEGORY6 + 1;
const unsigned int MAX_ENTROPY_TOKENS = DCT_EOB_TOKEN + 1;

const extern TreeArray< MAX_ENTROPY_TOKENS > token_tree;

#endif /* TOKENS_HH */
