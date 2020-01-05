#include <iostream>
#include "util.hpp"


namespace ceng391 {

int hamming_distance(const uchar* desc1, const uchar* desc2) {

    int distance = 0;
    for(int i = 0; i < 32; i++) {
        uchar result_xor = (desc1[i] ^ desc2[i]);

        while(result_xor) {
            uchar tmp = result_xor & 1;
            if(tmp == 1) {
                distance += 1;
            }
            result_xor = result_xor >> 1;
            
        } 
    }
    
    return distance;
}


}