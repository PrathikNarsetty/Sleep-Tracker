#include <iostream>
#include <vector>
#include <string>
using namespace std;
class Solution {
public:
    int minFlips(int a, int b, int c) {
     int d = a |b;
     d = d^c; // leaves me with all the differeing bits
     int e = a&b; // e equals all the same bits
     e = d&e; // e now equals all the differeing bits that are repeated. this num needs to be flipped in each
     e = __builtin_popcount(e); // e now equlas the number of set bits in e
     d = __builtin_popcount(d);
     return e+d;

    }
};