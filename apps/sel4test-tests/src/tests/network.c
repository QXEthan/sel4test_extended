#include <string.h>
#include <stdio.h>

#include "../test.h"
#include "../helpers.h"

static int test_network(struct env *env)
{
    int error;
    ZF_LOGI("############    In test_network, NETWORK_001   #################\n");


    test_eq(error, 0);

}
DEFINE_TEST(NETWORK_001, "Network Example", test_network, true)