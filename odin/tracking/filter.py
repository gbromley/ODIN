"""
This file contains filters for state estimation and prediction. 

"""
import numpy as np


class Filter(object):
    """
        Parameters
        ----------
        dim_x : int
            Number of state variables for the Kalman filter. For example, if
            you are tracking the position and velocity of an object in two
            dimensions, dim_x would be 4.
            This is used to set the default size of P, Q, and u

        dim_z : int
            Number of of measurement inputs. For example, if the sensor
            provides you with position in (x,y), dim_z would be 2.

        dim_u : int (optional)
            size of the control input, if it is being used.
            Default value of 0 indicates it is not used.

        compute_log_likelihood : bool (default = True)
            Computes log likelihood by default, but this can be a slow
            computation, so if you never use it you can turn this computation
            off.

        Attributes
        ----------
        x : numpy.array(dim_x, 1)
            Current state estimate. Any call to update() or predict() updates
            this variable.

        P : numpy.array(dim_x, dim_x)
            Current state covariance matrix. Any call to update() or predict()
            updates this variable.

        x_prior : numpy.array(dim_x, 1)
            Prior (predicted) state estimate. The *_prior and *_post attributes
            are for convenience; they store the  prior and posterior of the
            current epoch. Read Only.

        P_prior : numpy.array(dim_x, dim_x)
            Prior (predicted) state covariance matrix. Read Only.

        x_post : numpy.array(dim_x, 1)
            Posterior (updated) state estimate. Read Only.

        P_post : numpy.array(dim_x, dim_x)
            Posterior (updated) state covariance matrix. Read Only.

        z : numpy.array
            Last measurement used in update(). Read only.

        R : numpy.array(dim_z, dim_z)
            Measurement noise covariance matrix. Also known as the
            observation covariance.

        Q : numpy.array(dim_x, dim_x)
            Process noise covariance matrix. Also known as the transition
            covariance.

        F : numpy.array()
            State Transition matrix. Also known as `A` in some formulation.

        H : numpy.array(dim_z, dim_x)
            Measurement function. Also known as the observation matrix, or as `C`.

        y : numpy.array
            Residual of the update step. Read only.

        K : numpy.array(dim_x, dim_z)
            Kalman gain of the update step. Read only.

        S :  numpy.array
            System uncertainty (P projected to measurement space). Read only.

        SI :  numpy.array
            Inverse system uncertainty. Read only.

        log_likelihood : float
            log-likelihood of the last measurement. Read only.

        likelihood : float
            likelihood of last measurement. Read only.

            Computed from the log-likelihood. The log-likelihood can be very
            small,  meaning a large negative value such as -28000. Taking the
            exp() of that results in 0.0, which can break typical algorithms
            which multiply by this value, so by default we always return a
            number >= sys.float_info.min.

        mahalanobis : float
            mahalanobis distance of the innovation. Read only.

        inv : function, default numpy.linalg.inv
            If you prefer another inverse function, such as the Moore-Penrose
            pseudo inverse, set it to that instead: kf.inv = np.linalg.pinv

            This is only used to invert self.S. If you know it is diagonal, you
            might choose to set it to filterpy.common.inv_diagonal, which is
            several times faster than numpy.linalg.inv for diagonal matrices.

        alpha : float
            Fading memory setting. 1.0 gives the normal Kalman filter, and
            values slightly larger than 1.0 (such as 1.02) give a fading
            memory effect - previous measurements have less influence on the
            filter's estimates. This formulation of the Fading memory filter
            (there are many) is due to Dan Simon [1]_.

        References
        ----------

        .. [1] Dan Simon. "Optimal State Estimation." John Wiley & Sons.
        p. 208-212. (2006)

        .. [2] Roger Labbe. "Kalman and Bayesian Filters in Python"
        https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python

        """
    def __init__(self, order, initData, dt):
        if order > 2 or order < 0:
            raise('Not equipped to filter using jerks lol')
        
        if len(initData) < (order + 1) or len(initData) > (order +1):
            raise('Mismatch between order and initalization data')
        
        self.x = np.array(initData, copy=True)
        
    def predict():
        ### To Do ###
        return None
    
class GHFilter:
    """
        Parameters
        ----------
        dim_x : int
            Number of state variables for the Kalman filter. For example, if
            you are tracking the position and velocity of an object in two
            dimensions, dim_x would be 4.
            This is used to set the default size of P, Q, and u

        dim_z : int
            Number of of measurement inputs. For example, if the sensor
            provides you with position in (x,y), dim_z would be 2.

        dim_u : int (optional)
            size of the control input, if it is being used.
            Default value of 0 indicates it is not used.

        compute_log_likelihood : bool (default = True)
            Computes log likelihood by default, but this can be a slow
            computation, so if you never use it you can turn this computation
            off.

        Attributes
        ----------
        x : numpy.array(dim_x, 1)
            Current state estimate. Any call to update() or predict() updates
            this variable.

        P : numpy.array(dim_x, dim_x)
            Current state covariance matrix. Any call to update() or predict()
            updates this variable.

        x_prior : numpy.array(dim_x, 1)
            Prior (predicted) state estimate. The *_prior and *_post attributes
            are for convenience; they store the  prior and posterior of the
            current epoch. Read Only.

        P_prior : numpy.array(dim_x, dim_x)
            Prior (predicted) state covariance matrix. Read Only.

        x_post : numpy.array(dim_x, 1)
            Posterior (updated) state estimate. Read Only.

        P_post : numpy.array(dim_x, dim_x)
            Posterior (updated) state covariance matrix. Read Only.

        z : numpy.array
            Last measurement used in update(). Read only.

        R : numpy.array(dim_z, dim_z)
            Measurement noise covariance matrix. Also known as the
            observation covariance.

        Q : numpy.array(dim_x, dim_x)
            Process noise covariance matrix. Also known as the transition
            covariance.

        F : numpy.array()
            State Transition matrix. Also known as `A` in some formulation.

        H : numpy.array(dim_z, dim_x)
            Measurement function. Also known as the observation matrix, or as `C`.

        y : numpy.array
            Residual of the update step. Read only.

        K : numpy.array(dim_x, dim_z)
            Kalman gain of the update step. Read only.

        S :  numpy.array
            System uncertainty (P projected to measurement space). Read only.

        SI :  numpy.array
            Inverse system uncertainty. Read only.

        log_likelihood : float
            log-likelihood of the last measurement. Read only.

        likelihood : float
            likelihood of last measurement. Read only.

            Computed from the log-likelihood. The log-likelihood can be very
            small,  meaning a large negative value such as -28000. Taking the
            exp() of that results in 0.0, which can break typical algorithms
            which multiply by this value, so by default we always return a
            number >= sys.float_info.min.

        mahalanobis : float
            mahalanobis distance of the innovation. Read only.

        inv : function, default numpy.linalg.inv
            If you prefer another inverse function, such as the Moore-Penrose
            pseudo inverse, set it to that instead: kf.inv = np.linalg.pinv

            This is only used to invert self.S. If you know it is diagonal, you
            might choose to set it to filterpy.common.inv_diagonal, which is
            several times faster than numpy.linalg.inv for diagonal matrices.

        alpha : float
            Fading memory setting. 1.0 gives the normal Kalman filter, and
            values slightly larger than 1.0 (such as 1.02) give a fading
            memory effect - previous measurements have less influence on the
            filter's estimates. This formulation of the Fading memory filter
            (there are many) is due to Dan Simon [1]_.

        References
        ----------

        .. [1] Dan Simon. "Optimal State Estimation." John Wiley & Sons.
        p. 208-212. (2006)

        .. [2] Roger Labbe. "Kalman and Bayesian Filters in Python"
        https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python

        """
        
    def __init__(self, )
    
        