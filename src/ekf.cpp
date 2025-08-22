#include "ekf.hpp"

EKF::EKF(Eigen::Matrix2f &R_,
         Eigen::Matrix3f &Q_,
         Eigen::Matrix3f &P0_,
         Eigen::Vector3f &X0_,
         std::shared_ptr<Logger> &logger_)
    : R(R_), Q(Q_), P_hat(P0_), P(P0_), X(X0_), X_hat(X0_), logger(logger_)
{
}

void EKF::initLandmark(Measurement& Z, int& id)
{
    float &xr = X_hat(0);
    float &yr = X_hat(1);
    float &tr = X_hat(2);

    float &d = Z.range;
    float &b = Z.bearing;

    // Estimate landmark poses
    float mx = xr + d * cos(tr + b);
    float my = yr + d * sin(tr + b);

    Eigen::Vector2f m({mx,my});

    // Expand state matrix
    int old_size = X.size();
    int new_size = old_size + 2;  // Add 2 dims (mx,my)

    std::cout << "X old size:" << old_size << std::endl;
    std::cout << "X new size:" << new_size << std::endl;

    // X ve X_hat genişlet
    Eigen::VectorXf newX(new_size);
    newX.head(old_size) = X;
    newX.tail(2) = m;
    X = newX;

    Eigen::VectorXf newX_hat(new_size);
    newX_hat.head(old_size) = X_hat;
    newX_hat.tail(2) = m;
    X_hat = newX_hat;

    landmark_list.emplace_back(id);    

    std::cout << "Landmark init (x,y):" << mx << ", " << my << std::endl;

    // P ve P_hat matrislerini genişlet
    Eigen::MatrixXf newP(new_size, new_size);
    newP.setZero();
    newP.topLeftCorner(old_size, old_size) = P;
    newP.bottomRightCorner(2, 2) = Eigen::Matrix2f::Identity() * 99999.0f;
    P = newP;

    Eigen::MatrixXf newP_hat(new_size, new_size);
    newP_hat.setZero();
    newP_hat.topLeftCorner(old_size, old_size) = P_hat;
    newP_hat.bottomRightCorner(2, 2) = Eigen::Matrix2f::Identity() * 99999.0f;
    P_hat = newP_hat;
}

void EKF::predict(Eigen::Vector2f &U, double t)
{
    float& x_prev = X(0);
    float& y_prev = X(1);
    float& theta_prev = X(2);

    float& v = U(0);
    float& w = U(1);

    F << 1, 0, -v * sin(theta_prev),
        0, 1, v * cos(theta_prev),
        0, 0, 1;

    //Predict pose (Landmarks remain the same)
    X_hat(0) = x_prev + v * cos(theta_prev);
    X_hat(1) = y_prev + v * sin(theta_prev);
    X_hat(2) = theta_prev + w;

    //Compute P Matrix

    //Determine size
    const int R_size = 3;
    const int M_size = 2*landmark_list.size();
    const int P_size = R_size + M_size;

    std::cout << "P_hat size pred: " << P_hat.rows() << " x " << P_hat.cols() << std::endl;

    //Select local matrices
    Eigen::MatrixXf P_rr = P.topLeftCorner(R_size, R_size); //Robot covariance
    Eigen::MatrixXf P_rm = P.topRightCorner(R_size, M_size); //Robot and map covariance
    Eigen::MatrixXf P_mr = P.bottomLeftCorner(M_size, R_size); //Or just the transpose of P_rm
    Eigen::MatrixXf P_mm = P.bottomRightCorner(M_size, M_size); //Map cov

    Eigen::Matrix3f P_rr_hat = F * P_rr * F.transpose() + Q;
    Eigen::MatrixXf P_rm_hat = F * P_rm;
    Eigen::MatrixXf P_mr_hat = P_mr * F.transpose();
    Eigen::MatrixXf P_mm_hat = P_mm; // sabit kaldı


    //Now put them together again
    P_hat.topLeftCorner(R_size,R_size) =  P_rr_hat;
    P_hat.topRightCorner(R_size,M_size) = P_rm_hat;
    P_hat.bottomLeftCorner(M_size,R_size) =  P_mr_hat;
    P_hat.bottomRightCorner(M_size,M_size) =  P_mm_hat;

    std::vector<float> s;
    s.emplace_back(P_hat(0,0));
    s.emplace_back(P_hat(1,1));
    s.emplace_back(P_hat(0,1));

    std::cout << "EKF prediction: " << std::endl;
    std::cout << "x: " << X_hat(0) << " y: " << X_hat(1) << " Thet: " << X_hat(2) << std::endl;
    logger->logPosition("Prediction", Position(X_hat(0), X_hat(1), X_hat(2)),t,s);

    //Make X equal to X_hat, for no update situations
    X = X_hat;
    P = P_hat;
}

void EKF::update(const Measurement& Z, const int& id, double t)
{

    Eigen::Vector2f Z_vec(Z.range, Z.bearing);

    float& x = X_hat(0);
    float& y = X_hat(1);
    float& theta = X_hat(2);

    //Get landmark from state vector
    //Lm(0),x ---> X(3)
    //Lm(0),y ---> X(4)
    //Lm(1),x ---> X(5)
    //Lm(id),x --->X(3+2*id)
    //Lm(id),y --->X(3+2*id)

    //This is where the landmark is in state vector
    const int id_pos = 3 + 2*id;

    float& lm_x = X(id_pos);
    float& lm_y = X(id_pos + 1);

    float dx = lm_x - x;
    float dy = lm_y - y;

    float q = dx * dx + dy * dy;

    float range = std::sqrt(q);
    float bearing = std::atan2(dy, dx) - theta;

    // Normalize bearing to [-pi, pi]
    while (bearing > M_PI) bearing -= 2 * M_PI;
    while (bearing < -M_PI) bearing += 2 * M_PI;

    const int state_dim = 3 + 2*landmark_list.size();

    //Init. H matrix
    Eigen::MatrixXf H_temp(2, state_dim);
    H_temp.setZero();

    H = H_temp;

    std::cout << "state_dim = " << state_dim << std::endl;
    std::cout << "H size: " << H.rows() << " x " << H.cols() << std::endl;
    std::cout << "P_hat size: " << P_hat.rows() << " x " << P_hat.cols() << std::endl;
    std::cout << "R size: " << R.rows() << " x " << R.cols() << std::endl;
    std::cout << "id_pos = " << id_pos << std::endl;

    Eigen::Matrix<float,2,3> H_rr;

    // Compute H_rr matrix
    H_rr << -dx / sqrt(q), -dy / sqrt(q), 0,
        dy / q, -dx / q, -1;

    //Put H_rr to the beginning of the H matrix
    H.block<2,3>(0,0) = H_rr;

    //Compute H_mid for m with id
    Eigen::Matrix2f H_mi;
    H_mi << dx/range, dy/range,
           -dy/q,     dx/q;

    //Put H_mi to the corresponding position in H matrix
    H.block<2,2>(0,id_pos) = H_mi;




    S_in = H * P_hat * H.transpose() + R;
    Kt = P_hat * H.transpose() * S_in.inverse();

    Z_hat(0) = range;
    Z_hat(1) = bearing;

    X = X_hat + Kt * (Z_vec - Z_hat);
    P = P_hat - Kt * S_in * Kt.transpose();

    std::vector<float> s, sm;
    s.emplace_back(P(0,0));
    s.emplace_back(P(1,1));
    s.emplace_back(P(0,1));

    sm.emplace_back(P(id_pos,id_pos));
    sm.emplace_back(P(id_pos+1,id_pos+1));
    sm.emplace_back(P(id_pos,id_pos+1));

    std::cout << "EKF update: " << std::endl;
    std::cout << "x: " << X(0) << " y: " << X(1) << " Thet: " << X(2) << std::endl;
    logger->logPosition("Update", Position(X(0), X(1), X(2)),t,s);

    std::cout << "EKF landmark: " << std::endl;
    std::cout << "mx: " << X(id_pos) << " my: " << X(id_pos+1) << std::endl;
    logger->logPosition("Landmark EKF", Position(X(id_pos), X(id_pos+1), 0),t,sm);

    X_hat = X;
    P_hat = P;
}