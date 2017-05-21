#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

        VectorXd rmse(4);
        rmse << 0,0,0,0;

        VectorXd err(4);
        float n;
        n = 0;

        VectorXd res(4);
        res << 0, 0, 0, 0;
    if ((estimations.size() != 0) and (estimations.size() == ground_truth.size())) {


        //accumulate squared residuals
        for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        n = n + 1;
        err = estimations[i] - ground_truth[i];
        err =  err.array()*err.array();
            res = res + err;
        }


        //calculate the mean
        // ... your code here
    res = res.array()/n;
        //calculate the squared root
        // ... your code here
    rmse = res.array().sqrt();
        //return the result
    }
        return rmse;

    
}
