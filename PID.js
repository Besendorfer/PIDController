'use strict';

const CUR = 0;
const PREV = 1;

class PID {
	/*
	 * @param {Number} Kp			Proportional gain (to be tuned)
	 * @param {Number} Ki			Integral gain (to be tuned)
	 * @param {Number} Kd			Derivative gain (to be tuned)
	 * @param {Number} Kdd			Second Derivative gain (to be tuned)
	 */
	constructor(Kp = 0.300086, Ki = 0.00002, Kd = 20.079, Kdd = 0) {
		this.setParameters(Kp, Ki, Kd, Kdd);
	}

	setParameters(Kp = 0.300086, Ki = 0.00002, Kd = 20.079, Kdd = 0) {
		this.reset();
		this.Kp  = Kp;
		this.Ki  = Ki;
		this.Kd  = Kd;
		this.Kdd = Kdd;
	}

	/**
	 * Gets the desired velocity of the Sphero based on the distance from the next goal.
	 * Returns an object containing the x and y velocities for the Sphero.
	 *
	 * @param {Object} input			x and y distance of Sphero from goal (or next node)
	 * @return {Object}					Velocity to return (x and y)
	 */
	process(input) {
		if (this.Kp === -1)
			return {x: 20 * Math.sign(input.x), y: 20 * Math.sign(input.y)};

		this.unshiftErr(input);

		let px = this.Kp * this.err[CUR].x;
		let ix = this.Ki * this.sumErr.x;
		let dx = this.Kd * this.de(CUR).x / this.dt(CUR);
		let ddx = this.Kd * this.de(PREV).x / this.dt(PREV);
		let velX = px + ix + dx + ddx;

		let py = this.Kp *  this.err[CUR].y;
		let iy = this.Ki * this.sumErr.y;
		let dy = this.Kd * this.de(CUR).y / this.dt(CUR);
		let ddy = this.Kd * this.de(PREV).y / this.dt(PREV);
		let velY = py + iy + dy + ddy;

		return {x: velX, y: velY};
	}

	reset() {
		this.err = [{x: 0, y: 0}, {x: 0, y: 0}, {x: 0, y: 0}];
		this.time = [Date.now(),
					Date.now(),
					Date.now()];
		this.sumErr = {x: 0, y: 0};
	}

	unshiftNow() {
		this.time.pop();
		this.time.unshift(Date.now());
	}

	unshiftErr(e) {
		this.unshiftNow();
		this.err.pop();
		this.err.unshift(e);
		this.sumErr.x += this.dt(CUR) * this.err[CUR].x;
		this.sumErr.y += this.dt(CUR) * this.err[CUR].y;
	}

	dt(t) {
		return Math.max(35, this.time[t] - this.time[t + 1]);
	}

	de(t) {
		return {
			x: this.err[t].x - this.err[t + 1].x,
			y: this.err[t].y - this.err[t + 1].y
		};
	}
}

module.exports = PID;
