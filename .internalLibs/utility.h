#ifndef UTILITY_H
#define UTILITY_H
#include <iostream>
#include <iomanip>

//Progress function
void showProgress(btScalar simTime, btScalar totalTime){
    float progress = simTime/totalTime;
    int barWidth = 70;

    std::cout << "[";
    int pos = floor(barWidth * progress);
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << std::fixed << std::setprecision(2) << (progress * 100.0) << " %\r";
    std::cout.flush();
    
    if (simTime==totalTime){
    	std::cout << std::endl;
    }
}

#endif