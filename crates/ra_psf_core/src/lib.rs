use osqp::{CscMatrix, Problem, Settings, Status};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

const STATE_DIM: usize = 5;
const CONTROL_DIM: usize = 2;
const DISTURBANCE_DIM: usize = 3;

fn vector<const N: usize>(value: Vec<f64>, name: &str) -> PyResult<[f64; N]> {
    if value.len() != N || value.iter().any(|item| !item.is_finite()) {
        return Err(PyValueError::new_err(format!(
            "{name} must contain {N} finite values"
        )));
    }
    let mut result = [0.0; N];
    result.copy_from_slice(&value);
    Ok(result)
}

fn lag_coefficients(dt: f64, tau: f64) -> (f64, f64) {
    let decay = (-dt / tau).exp();
    (decay, 1.0 - decay)
}

fn pose_increment(theta: f64, v: f64, lateral: f64, yaw_rate: f64, dt: f64) -> (f64, f64) {
    let theta_next = theta + yaw_rate * dt;
    if yaw_rate.abs() < 1.0e-8 {
        return (
            dt * (v * theta.cos() - lateral * theta.sin()),
            dt * (v * theta.sin() + lateral * theta.cos()),
        );
    }
    let sin_delta = theta_next.sin() - theta.sin();
    let cos_delta = theta_next.cos() - theta.cos();
    (
        (v * sin_delta + lateral * cos_delta) / yaw_rate,
        (v * (theta.cos() - theta_next.cos()) + lateral * sin_delta) / yaw_rate,
    )
}

fn pose_partials(theta: f64, v: f64, lateral: f64, yaw_rate: f64, dt: f64) -> ([f64; 3], [f64; 3]) {
    let theta_next = theta + yaw_rate * dt;
    if yaw_rate.abs() < 1.0e-8 {
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();
        return (
            [
                dt * (-v * sin_theta - lateral * cos_theta),
                dt * cos_theta,
                0.5 * dt * dt * (-v * sin_theta - lateral * cos_theta),
            ],
            [
                dt * (v * cos_theta - lateral * sin_theta),
                dt * sin_theta,
                0.5 * dt * dt * (v * cos_theta - lateral * sin_theta),
            ],
        );
    }

    let sin_theta = theta.sin();
    let cos_theta = theta.cos();
    let sin_next = theta_next.sin();
    let cos_next = theta_next.cos();
    let inverse = 1.0 / yaw_rate;
    let numerator_x = v * (sin_next - sin_theta) + lateral * (cos_next - cos_theta);
    let numerator_y = v * (cos_theta - cos_next) + lateral * (sin_next - sin_theta);
    let numerator_x_rate = dt * (v * cos_next - lateral * sin_next);
    let numerator_y_rate = dt * (v * sin_next + lateral * cos_next);
    (
        [
            inverse * (v * (cos_next - cos_theta) + lateral * (-sin_next + sin_theta)),
            (sin_next - sin_theta) * inverse,
            (numerator_x_rate * yaw_rate - numerator_x) * inverse * inverse,
        ],
        [
            inverse * (v * (sin_next - sin_theta) + lateral * (cos_next - cos_theta)),
            (cos_theta - cos_next) * inverse,
            (numerator_y_rate * yaw_rate - numerator_y) * inverse * inverse,
        ],
    )
}

fn model_step(
    state: [f64; STATE_DIM],
    control: [f64; CONTROL_DIM],
    disturbance: [f64; DISTURBANCE_DIM],
    dt: f64,
    tau_v: f64,
    tau_omega: f64,
) -> [f64; STATE_DIM] {
    let (decay_v, gain_v) = lag_coefficients(dt, tau_v);
    let (decay_omega, gain_omega) = lag_coefficients(dt, tau_omega);
    let (delta_x, delta_y) = pose_increment(
        state[2],
        state[3],
        disturbance[1],
        state[4] + disturbance[2],
        dt,
    );
    [
        state[0] + delta_x,
        state[1] + delta_y,
        state[2] + (state[4] + disturbance[2]) * dt,
        decay_v * state[3] + gain_v * control[0] + tau_v * gain_v * disturbance[0],
        decay_omega * state[4] + gain_omega * control[1],
    ]
}

fn model_jacobians(
    state: [f64; STATE_DIM],
    disturbance: [f64; DISTURBANCE_DIM],
    dt: f64,
    tau_v: f64,
    tau_omega: f64,
) -> (
    [[f64; STATE_DIM]; STATE_DIM],
    [[f64; CONTROL_DIM]; STATE_DIM],
) {
    let (pose_x, pose_y) = pose_partials(
        state[2],
        state[3],
        disturbance[1],
        state[4] + disturbance[2],
        dt,
    );
    let (decay_v, gain_v) = lag_coefficients(dt, tau_v);
    let (decay_omega, gain_omega) = lag_coefficients(dt, tau_omega);
    let mut a = [[0.0; STATE_DIM]; STATE_DIM];
    a[0][0] = 1.0;
    a[1][1] = 1.0;
    a[0][2] = pose_x[0];
    a[0][3] = pose_x[1];
    a[0][4] = pose_x[2];
    a[1][2] = pose_y[0];
    a[1][3] = pose_y[1];
    a[1][4] = pose_y[2];
    a[2][2] = 1.0;
    a[2][4] = dt;
    a[3][3] = decay_v;
    a[4][4] = decay_omega;

    let mut b = [[0.0; CONTROL_DIM]; STATE_DIM];
    b[3][0] = gain_v;
    b[4][1] = gain_omega;
    (a, b)
}

#[pyfunction]
fn step(
    state: Vec<f64>,
    control: Vec<f64>,
    disturbance: Vec<f64>,
    dt: f64,
    tau_v: f64,
    tau_omega: f64,
) -> PyResult<Vec<f64>> {
    if dt <= 0.0 || tau_v <= 0.0 || tau_omega <= 0.0 {
        return Err(PyValueError::new_err(
            "dt and time constants must be positive",
        ));
    }
    let x = vector::<STATE_DIM>(state, "state")?;
    let u = vector::<CONTROL_DIM>(control, "control")?;
    let d = vector::<DISTURBANCE_DIM>(disturbance, "disturbance")?;
    Ok(model_step(x, u, d, dt, tau_v, tau_omega).to_vec())
}

#[pyfunction]
fn jacobians(
    state: Vec<f64>,
    disturbance: Vec<f64>,
    dt: f64,
    tau_v: f64,
    tau_omega: f64,
) -> PyResult<(Vec<Vec<f64>>, Vec<Vec<f64>>)> {
    if dt <= 0.0 || tau_v <= 0.0 || tau_omega <= 0.0 {
        return Err(PyValueError::new_err(
            "dt and time constants must be positive",
        ));
    }
    let x = vector::<STATE_DIM>(state, "state")?;
    let d = vector::<DISTURBANCE_DIM>(disturbance, "disturbance")?;
    let (a, b) = model_jacobians(x, d, dt, tau_v, tau_omega);
    Ok((
        a.iter().map(|row| row.to_vec()).collect(),
        b.iter().map(|row| row.to_vec()).collect(),
    ))
}

#[pyfunction]
fn rollout(
    state: Vec<f64>,
    controls: Vec<Vec<f64>>,
    disturbance: Vec<f64>,
    dt: f64,
    tau_v: f64,
    tau_omega: f64,
) -> PyResult<Vec<Vec<f64>>> {
    if dt <= 0.0 || tau_v <= 0.0 || tau_omega <= 0.0 {
        return Err(PyValueError::new_err(
            "dt and time constants must be positive",
        ));
    }
    let mut current = vector::<STATE_DIM>(state, "state")?;
    let d = vector::<DISTURBANCE_DIM>(disturbance, "disturbance")?;
    let mut result = vec![current.to_vec()];
    for control in controls {
        let u = vector::<CONTROL_DIM>(control, "control")?;
        current = model_step(current, u, d, dt, tau_v, tau_omega);
        result.push(current.to_vec());
    }
    Ok(result)
}

#[pyfunction]
fn cvar_multiplier(alpha: f64) -> PyResult<f64> {
    if !(0.5..1.0).contains(&alpha) {
        return Err(PyValueError::new_err("alpha must be in (0.5, 1.0)"));
    }
    let mut low = -10.0;
    let mut high = 10.0;
    for _ in 0..80 {
        let mid = 0.5 * (low + high);
        let cdf = 0.5 * (1.0 + erf(mid / 2.0_f64.sqrt()));
        if cdf < alpha {
            low = mid;
        } else {
            high = mid;
        }
    }
    let quantile = 0.5 * (low + high);
    let density = (-0.5 * quantile * quantile).exp() / (2.0 * std::f64::consts::PI).sqrt();
    Ok(density / (1.0 - alpha))
}

#[pyclass]
struct HorizonQpWorkspace {
    problem: Problem,
    horizon: usize,
    max_obstacles: usize,
    nvar: usize,
    ncon: usize,
    du_offset: usize,
    slack_offset: usize,
    rate_penalty: [f64; CONTROL_DIM],
    max_control_delta: [f64; CONTROL_DIM],
    v_min: f64,
    v_max: f64,
    omega_min: f64,
    omega_max: f64,
    last_x: Vec<f64>,
    last_y: Vec<f64>,
}

impl HorizonQpWorkspace {
    fn dx_index(&self, step: usize, component: usize) -> usize {
        step * STATE_DIM + component
    }

    fn du_index(&self, step: usize, component: usize) -> usize {
        self.du_offset + step * CONTROL_DIM + component
    }

    fn slack_index(&self, step: usize, obstacle: usize) -> usize {
        self.slack_offset + step * self.max_obstacles + obstacle
    }

    fn constraint_row_count(horizon: usize, max_obstacles: usize) -> usize {
        STATE_DIM
            + horizon * STATE_DIM
            + horizon * CONTROL_DIM
            + CONTROL_DIM
            + 2 * horizon * max_obstacles
    }

    fn dense_constraint_matrix(&self) -> Vec<f64> {
        let mut matrix = vec![0.0; self.ncon * self.nvar];
        let mut row = 0;

        for component in 0..STATE_DIM {
            matrix[row * self.nvar + self.dx_index(0, component)] = 1.0;
            row += 1;
        }

        for step in 0..self.horizon {
            for component in 0..STATE_DIM {
                matrix[row * self.nvar + self.dx_index(step + 1, component)] = 1.0;
                row += 1;
            }
        }

        for step in 0..self.horizon {
            for component in 0..CONTROL_DIM {
                matrix[row * self.nvar + self.du_index(step, component)] = 1.0;
                row += 1;
            }
        }

        for component in 0..CONTROL_DIM {
            matrix[row * self.nvar + self.du_index(0, component)] = 1.0;
            row += 1;
        }

        for step in 0..self.horizon {
            for obstacle in 0..self.max_obstacles {
                matrix[row * self.nvar + self.dx_index(step, 0)] = 0.0;
                matrix[row * self.nvar + self.dx_index(step, 1)] = 0.0;
                matrix[row * self.nvar + self.slack_index(step, obstacle)] = 1.0;
                row += 1;
            }
        }

        for step in 0..self.horizon {
            for obstacle in 0..self.max_obstacles {
                matrix[row * self.nvar + self.slack_index(step, obstacle)] = 1.0;
                row += 1;
            }
        }

        debug_assert_eq!(row, self.ncon);
        matrix
    }
}

#[pymethods]
impl HorizonQpWorkspace {
    #[new]
    #[pyo3(signature = (horizon, max_obstacles, q_diag, r_diag, terminal_cost, r_filter_diag, rate_penalty_diag, max_control_delta, slack_penalty, v_min, v_max, omega_min, omega_max))]
    fn new(
        horizon: usize,
        max_obstacles: usize,
        q_diag: Vec<f64>,
        r_diag: Vec<f64>,
        terminal_cost: Vec<Vec<f64>>,
        r_filter_diag: Vec<f64>,
        rate_penalty_diag: Vec<f64>,
        max_control_delta: Vec<f64>,
        slack_penalty: f64,
        v_min: f64,
        v_max: f64,
        omega_min: f64,
        omega_max: f64,
    ) -> PyResult<Self> {
        if horizon == 0 || max_obstacles == 0 {
            return Err(PyValueError::new_err(
                "horizon and max_obstacles must be positive",
            ));
        }
        if q_diag.len() != STATE_DIM
            || r_diag.len() != CONTROL_DIM
            || r_filter_diag.len() != CONTROL_DIM
            || rate_penalty_diag.len() != CONTROL_DIM
            || max_control_delta.len() != CONTROL_DIM
            || rate_penalty_diag
                .iter()
                .any(|value| !value.is_finite() || *value <= 0.0)
            || max_control_delta
                .iter()
                .any(|value| !value.is_finite() || *value <= 0.0)
            || q_diag
                .iter()
                .any(|value| !value.is_finite() || *value <= 0.0)
            || r_diag
                .iter()
                .any(|value| !value.is_finite() || *value <= 0.0)
            || r_filter_diag
                .iter()
                .any(|value| !value.is_finite() || *value <= 0.0)
        {
            return Err(PyValueError::new_err(
                "QP weights have invalid dimensions or values",
            ));
        }
        if terminal_cost.len() != STATE_DIM
            || terminal_cost
                .iter()
                .any(|row| row.len() != STATE_DIM || row.iter().any(|value| !value.is_finite()))
        {
            return Err(PyValueError::new_err(
                "terminal_cost must be a finite 5 by 5 matrix",
            ));
        }
        if !slack_penalty.is_finite()
            || slack_penalty <= 0.0
            || !v_min.is_finite()
            || !v_max.is_finite()
            || !omega_min.is_finite()
            || !omega_max.is_finite()
            || v_min > v_max
            || omega_min > omega_max
        {
            return Err(PyValueError::new_err("invalid QP bounds or slack penalty"));
        }

        let dx_vars = (horizon + 1) * STATE_DIM;
        let du_offset = dx_vars;
        let slack_offset = du_offset + horizon * CONTROL_DIM;
        let nvar = slack_offset + horizon * max_obstacles;
        let ncon = Self::constraint_row_count(horizon, max_obstacles);

        let mut p_dense = vec![0.0; nvar * nvar];
        for step in 0..horizon {
            for component in 0..STATE_DIM {
                let index = step * STATE_DIM + component;
                p_dense[index * nvar + index] = 2.0 * q_diag[component];
            }
        }
        for row in 0..STATE_DIM {
            for column in 0..STATE_DIM {
                let row_index = horizon * STATE_DIM + row;
                let column_index = horizon * STATE_DIM + column;
                p_dense[row_index * nvar + column_index] = 2.0 * terminal_cost[row][column];
            }
        }
        for step in 0..horizon {
            for component in 0..CONTROL_DIM {
                let index = du_offset + step * CONTROL_DIM + component;
                let filter_weight = if step == 0 {
                    r_filter_diag[component]
                } else {
                    0.0
                };
                let rate_weight = if step == 0 {
                    rate_penalty_diag[component]
                } else {
                    0.0
                };
                p_dense[index * nvar + index] =
                    2.0 * (r_diag[component] + filter_weight + rate_weight);
            }
            for obstacle in 0..max_obstacles {
                let index = slack_offset + step * max_obstacles + obstacle;
                p_dense[index * nvar + index] = 2.0 * slack_penalty;
            }
        }

        let p = CscMatrix::from_row_iter_dense(nvar, nvar, p_dense).into_upper_tri();
        let initial_a = CscMatrix::from_row_iter_dense(ncon, nvar, {
            let mut matrix = vec![0.0; ncon * nvar];
            let mut row = 0;
            for component in 0..STATE_DIM {
                matrix[row * nvar + component] = 1.0;
                row += 1;
            }
            for step in 0..horizon {
                for component in 0..STATE_DIM {
                    matrix[row * nvar + (step + 1) * STATE_DIM + component] = 1.0;
                    row += 1;
                }
            }
            for step in 0..horizon {
                for component in 0..CONTROL_DIM {
                    matrix[row * nvar + du_offset + step * CONTROL_DIM + component] = 1.0;
                    row += 1;
                }
            }
            for component in 0..CONTROL_DIM {
                matrix[row * nvar + du_offset + component] = 1.0;
                row += 1;
            }
            for step in 0..horizon {
                for obstacle in 0..max_obstacles {
                    matrix[row * nvar + slack_offset + step * max_obstacles + obstacle] = 1.0;
                    row += 1;
                }
            }
            for step in 0..horizon {
                for obstacle in 0..max_obstacles {
                    matrix[row * nvar + slack_offset + step * max_obstacles + obstacle] = 1.0;
                    row += 1;
                }
            }
            matrix
        });
        let q = vec![0.0; nvar];
        let lower = vec![-1.0e30; ncon];
        let upper = vec![1.0e30; ncon];
        let settings = Settings::default()
            .verbose(false)
            .adaptive_rho(true)
            .polishing(true)
            .eps_abs(2.0e-5)
            .eps_rel(2.0e-5)
            .max_iter(10000);
        let problem = Problem::new(p, &q, initial_a, &lower, &upper, &settings)
            .map_err(|error| PyValueError::new_err(format!("OSQP setup failed: {error:?}")))?;

        Ok(Self {
            problem,
            horizon,
            max_obstacles,
            nvar,
            ncon,
            du_offset,
            slack_offset,
            rate_penalty: [rate_penalty_diag[0], rate_penalty_diag[1]],
            max_control_delta: [max_control_delta[0], max_control_delta[1]],
            v_min,
            v_max,
            omega_min,
            omega_max,
            last_x: Vec::new(),
            last_y: Vec::new(),
        })
    }

    fn solve(
        &mut self,
        x0_error: Vec<f64>,
        previous_control: Vec<f64>,
        u_reference: Vec<Vec<f64>>,
        a_sequence: Vec<Vec<Vec<f64>>>,
        b_sequence: Vec<Vec<Vec<f64>>>,
        obstacle_normals: Vec<Vec<Vec<f64>>>,
        obstacle_rhs: Vec<Vec<f64>>,
    ) -> PyResult<(Vec<f64>, String, f64)> {
        if x0_error.len() != STATE_DIM || x0_error.iter().any(|value| !value.is_finite()) {
            return Err(PyValueError::new_err(
                "x0_error must contain five finite values",
            ));
        }
        if previous_control.len() != CONTROL_DIM
            || previous_control.iter().any(|value| !value.is_finite())
        {
            return Err(PyValueError::new_err(
                "previous_control must contain two finite values",
            ));
        }
        if u_reference.len() != self.horizon
            || a_sequence.len() != self.horizon
            || b_sequence.len() != self.horizon
            || obstacle_normals.len() != self.horizon
            || obstacle_rhs.len() != self.horizon
        {
            return Err(PyValueError::new_err(
                "QP sequence lengths do not match horizon",
            ));
        }

        let mut matrix = self.dense_constraint_matrix();
        let mut lower = vec![-1.0e30; self.ncon];
        let mut upper = vec![1.0e30; self.ncon];
        let mut row = 0;

        for component in 0..STATE_DIM {
            lower[row] = x0_error[component];
            upper[row] = x0_error[component];
            row += 1;
        }

        for step in 0..self.horizon {
            if u_reference[step].len() != CONTROL_DIM {
                return Err(PyValueError::new_err(
                    "each control reference must have length two",
                ));
            }
            if a_sequence[step].len() != STATE_DIM
                || a_sequence[step].iter().any(|line| {
                    line.len() != STATE_DIM || line.iter().any(|value| !value.is_finite())
                })
            {
                return Err(PyValueError::new_err(
                    "each A matrix must be 5 by 5 and finite",
                ));
            }
            if b_sequence[step].len() != STATE_DIM
                || b_sequence[step].iter().any(|line| {
                    line.len() != CONTROL_DIM || line.iter().any(|value| !value.is_finite())
                })
            {
                return Err(PyValueError::new_err(
                    "each B matrix must be 5 by 2 and finite",
                ));
            }
            if u_reference[step].iter().any(|value| !value.is_finite()) {
                return Err(PyValueError::new_err("control references must be finite"));
            }

            for component in 0..STATE_DIM {
                for column in 0..STATE_DIM {
                    matrix[row * self.nvar + self.dx_index(step, column)] =
                        -a_sequence[step][component][column];
                }
                for column in 0..CONTROL_DIM {
                    matrix[row * self.nvar + self.du_index(step, column)] =
                        -b_sequence[step][component][column];
                }
                matrix[row * self.nvar + self.dx_index(step + 1, component)] = 1.0;
                lower[row] = 0.0;
                upper[row] = 0.0;
                row += 1;
            }
        }

        for step in 0..self.horizon {
            lower[row] = self.v_min - u_reference[step][0];
            upper[row] = self.v_max - u_reference[step][0];
            row += 1;
            lower[row] = self.omega_min - u_reference[step][1];
            upper[row] = self.omega_max - u_reference[step][1];
            row += 1;
        }

        for component in 0..CONTROL_DIM {
            lower[row] = previous_control[component]
                - self.max_control_delta[component]
                - u_reference[0][component];
            upper[row] = previous_control[component] + self.max_control_delta[component]
                - u_reference[0][component];
            row += 1;
        }

        for step in 0..self.horizon {
            if obstacle_normals[step].len() != self.max_obstacles
                || obstacle_rhs[step].len() != self.max_obstacles
            {
                return Err(PyValueError::new_err(
                    "obstacle normals and bounds do not match max_obstacles",
                ));
            }
            for obstacle in 0..self.max_obstacles {
                let normal = &obstacle_normals[step][obstacle];
                if normal.len() != 2 || normal.iter().any(|value| !value.is_finite()) {
                    return Err(PyValueError::new_err(
                        "each obstacle normal must have length two",
                    ));
                }
                if !obstacle_rhs[step][obstacle].is_finite() {
                    return Err(PyValueError::new_err("obstacle bounds must be finite"));
                }
                matrix[row * self.nvar + self.dx_index(step, 0)] = normal[0];
                matrix[row * self.nvar + self.dx_index(step, 1)] = normal[1];
                lower[row] = obstacle_rhs[step][obstacle];
                upper[row] = 1.0e30;
                row += 1;
            }
        }

        for _ in 0..self.horizon {
            for _ in 0..self.max_obstacles {
                lower[row] = 0.0;
                upper[row] = 1.0e30;
                row += 1;
            }
        }
        debug_assert_eq!(row, self.ncon);

        let a = CscMatrix::from_row_iter_dense(self.ncon, self.nvar, matrix);
        let mut q = vec![0.0; self.nvar];
        for component in 0..CONTROL_DIM {
            q[self.du_index(0, component)] = 2.0
                * self.rate_penalty[component]
                * (u_reference[0][component] - previous_control[component]);
        }
        self.problem.update_A(a);
        self.problem.update_lin_cost(&q);
        self.problem.update_bounds(&lower, &upper);
        if self.last_x.len() == self.nvar && self.last_y.len() == self.ncon {
            self.problem.warm_start(&self.last_x, &self.last_y);
        }

        let status = self.problem.solve();
        let solve_time_ms = status.solve_time().as_secs_f64() * 1000.0;
        let (status_name, solution): (String, Option<(Vec<f64>, Vec<f64>)>) = match status {
            Status::Solved(solution) => (
                "solved".to_string(),
                Some((solution.x().to_vec(), solution.y().to_vec())),
            ),
            Status::SolvedInaccurate(solution) => (
                "solved_inaccurate".to_string(),
                Some((solution.x().to_vec(), solution.y().to_vec())),
            ),
            Status::MaxIterationsReached(solution) => (
                "max_iterations".to_string(),
                Some((solution.x().to_vec(), solution.y().to_vec())),
            ),
            Status::TimeLimitReached(solution) => (
                "time_limit".to_string(),
                Some((solution.x().to_vec(), solution.y().to_vec())),
            ),
            other => (format!("{other:?}"), None),
        };

        if let Some((x, y)) = solution {
            self.last_x = x.clone();
            self.last_y = y;
            Ok((x, status_name, solve_time_ms))
        } else {
            Ok((self.last_x.clone(), status_name, solve_time_ms))
        }
    }

    fn reset(&mut self) {
        self.last_x.clear();
        self.last_y.clear();
    }
}

#[pyfunction]
fn solve_step_qp(
    nominal_control: Vec<f64>,
    hessian_diag: Vec<f64>,
    linear_term: Vec<f64>,
    lower_delta: Vec<f64>,
    upper_delta: Vec<f64>,
    barrier_normals: Vec<Vec<f64>>,
    barrier_bounds: Vec<f64>,
) -> PyResult<(Vec<f64>, String, f64)> {
    let nominal = vector::<CONTROL_DIM>(nominal_control, "nominal_control")?;
    let hessian = vector::<CONTROL_DIM>(hessian_diag, "hessian_diag")?;
    let q = vector::<CONTROL_DIM>(linear_term, "linear_term")?;
    let lower = vector::<CONTROL_DIM>(lower_delta, "lower_delta")?;
    let upper = vector::<CONTROL_DIM>(upper_delta, "upper_delta")?;
    if hessian.iter().any(|value| *value <= 0.0) {
        return Err(PyValueError::new_err("hessian diagonal must be positive"));
    }
    if lower.iter().zip(upper.iter()).any(|(lo, hi)| lo > hi) {
        return Err(PyValueError::new_err(
            "lower_delta must not exceed upper_delta",
        ));
    }
    if barrier_normals.len() != barrier_bounds.len() {
        return Err(PyValueError::new_err(
            "barrier_normals and barrier_bounds must have equal lengths",
        ));
    }

    let mut rows: Vec<Vec<f64>> = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
    let mut lower_bounds = vec![lower[0], lower[1]];
    let mut upper_bounds = vec![upper[0], upper[1]];
    for (normal, bound) in barrier_normals.iter().zip(barrier_bounds.iter()) {
        if normal.len() != CONTROL_DIM || normal.iter().any(|value| !value.is_finite()) {
            return Err(PyValueError::new_err(
                "each barrier normal must contain two finite values",
            ));
        }
        rows.push(normal.clone());
        lower_bounds.push(*bound);
        upper_bounds.push(1.0e30);
    }

    let p_dense = [[hessian[0], 0.0], [0.0, hessian[1]]];
    let p = CscMatrix::from(&p_dense).into_upper_tri();
    let a = CscMatrix::from(&rows);
    let settings = Settings::default()
        .verbose(false)
        .adaptive_rho(false)
        .polishing(true)
        .eps_abs(1.0e-6)
        .eps_rel(1.0e-6)
        .max_iter(2000);
    let mut problem = Problem::new(p, &q, a, &lower_bounds, &upper_bounds, &settings)
        .map_err(|error| PyValueError::new_err(format!("OSQP setup failed: {error:?}")))?;
    let result = problem.solve();
    let solve_time_ms = result.solve_time().as_secs_f64() * 1000.0;
    match result {
        Status::Solved(solution) | Status::SolvedInaccurate(solution) => {
            let delta = solution.x().to_vec();
            Ok((
                vec![nominal[0] + delta[0], nominal[1] + delta[1]],
                "solved".to_string(),
                solve_time_ms,
            ))
        }
        other => Ok((nominal.to_vec(), format!("{other:?}"), solve_time_ms)),
    }
}

fn erf(value: f64) -> f64 {
    let sign = if value < 0.0 { -1.0 } else { 1.0 };
    let x = value.abs();
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let polynomial = (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736)
        * t
        + 0.254829592)
        * t;
    sign * (1.0 - polynomial * (-x * x).exp())
}

#[pymodule]
fn ra_psf_rs(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(step, module)?)?;
    module.add_function(wrap_pyfunction!(jacobians, module)?)?;
    module.add_function(wrap_pyfunction!(rollout, module)?)?;
    module.add_function(wrap_pyfunction!(cvar_multiplier, module)?)?;
    module.add_function(wrap_pyfunction!(solve_step_qp, module)?)?;
    module.add_class::<HorizonQpWorkspace>()?;
    Ok(())
}
