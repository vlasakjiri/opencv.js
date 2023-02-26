var kalmanFilter;kalmanFilter =
/******/ (() => { // webpackBootstrap
/******/ 	var __webpack_modules__ = ({

/***/ "./index.js":
/*!******************!*\
  !*** ./index.js ***!
  \******************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const modelCollection = __webpack_require__(/*! ./lib/model-collection */ "./lib/model-collection.js");

module.exports = {
	registerDynamic: modelCollection.registerDynamic,
	KalmanFilter: __webpack_require__(/*! ./lib/kalman-filter */ "./lib/kalman-filter.js"),
	registerObservation: modelCollection.registerObservation,
	buildObservation: modelCollection.buildObservation,
	buildDynamic: modelCollection.buildDynamic,
	getCovariance: __webpack_require__(/*! ./lib/utils/get-covariance */ "./lib/utils/get-covariance.js"),
	State: __webpack_require__(/*! ./lib/state */ "./lib/state.js"),
	checkCovariance: __webpack_require__(/*! ./lib/utils/check-covariance */ "./lib/utils/check-covariance.js"),
	correlationToCovariance: __webpack_require__(/*! ./lib/utils/correlation-to-covariance */ "./lib/utils/correlation-to-covariance.js"),
	covarianceToCorrelation: __webpack_require__(/*! ./lib/utils/covariance-to-correlation */ "./lib/utils/covariance-to-correlation.js"),
	linalgebra: __webpack_require__(/*! ./lib/linalgebra */ "./lib/linalgebra/index.js")
};


/***/ }),

/***/ "./lib/core-kalman-filter.js":
/*!***********************************!*\
  !*** ./lib/core-kalman-filter.js ***!
  \***********************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const matMul = __webpack_require__(/*! ../lib/linalgebra/mat-mul.js */ "./lib/linalgebra/mat-mul.js");
const transpose = __webpack_require__(/*! ../lib/linalgebra/transpose.js */ "./lib/linalgebra/transpose.js");
const add = __webpack_require__(/*! ../lib/linalgebra/add.js */ "./lib/linalgebra/add.js");
const invert = __webpack_require__(/*! ../lib/linalgebra/invert.js */ "./lib/linalgebra/invert.js");
const sub = __webpack_require__(/*! ../lib/linalgebra/sub.js */ "./lib/linalgebra/sub.js");
const getIdentity = __webpack_require__(/*! ../lib/linalgebra/identity.js */ "./lib/linalgebra/identity.js");
const State = __webpack_require__(/*! ./state.js */ "./lib/state.js");
const checkMatrix = __webpack_require__(/*! ./utils/check-matrix.js */ "./lib/utils/check-matrix.js");
/**
* @callback ObservationCallback
* @param {Object} opts
* @param {Number} opts.index
* @param {Number} opts.previousCorrected
*/

/**
* @typedef {Object} ObservationConfig
* @property {Number} dimension
* @property {Array.Array.<Number>> | ObservationCallback} stateProjection,
* @property {Array.Array.<Number>> | ObservationCallback} covariance
*/

/**
* @callback DynamicCallback
* @param {Object} opts
* @param {Number} opts.index
* @param {State} opts.predicted
* @param {Observation} opts.observation
*/

/**
* @typedef {Object} DynamicConfig
* @property {Number} dimension
* @property {Array.Array.<Number>> | DynamicCallback} transition,
* @property {Array.Array.<Number>> | DynamicCallback} covariance
*/

const defaultLogger = {
	info: (...args) => console.log(...args),
	debug: () => {},
	warn: (...args) => console.log(...args),
	error: (...args) => console.log(...args)
};

/**
* @class
* @property {DynamicConfig} dynamic the system's dynamic model
* @property {ObservationConfig} observation the system's observation model
*@property logger a Winston-like logger
*/
class CoreKalmanFilter {
	/**
	* @param {DynamicConfig} dynamic
	* @param {ObservationConfig} observation the system's observation model
	*/

	constructor({dynamic, observation, logger = defaultLogger}) {
		this.dynamic = dynamic;
		this.observation = observation;
		this.logger = logger;
	}

	getValue(fn, options) {
		return (typeof (fn) === 'function' ? fn(options) : fn);
	}

	getInitState() {
		const {mean: meanInit, covariance: covarianceInit, index: indexInit} = this.dynamic.init;
		const initState = new State({
			mean: meanInit,
			covariance: covarianceInit,
			index: indexInit
		});
		return initState;
	}

	/**
	This will return the predicted covariance of a given previousCorrected State, this will help us to build the asymptoticState.
	* @param {State} previousCorrected
	* @returns{Array.<Array.<Number>>}
	*/

	getPredictedCovariance(options = {}) {
		let {previousCorrected, index} = options;
		previousCorrected = previousCorrected || this.getInitState();

		const getValueOptions = Object.assign({}, {previousCorrected, index}, options);
		const d = this.getValue(this.dynamic.transition, getValueOptions);
		const dTransposed = transpose(d);
		const covarianceInter = matMul(d, previousCorrected.covariance);
		const covariancePrevious = matMul(covarianceInter, dTransposed);
		const dynCov = this.getValue(this.dynamic.covariance, getValueOptions);

		const covariance = add(
			dynCov,
			covariancePrevious
		);
		checkMatrix(covariance, [this.dynamic.dimension, this.dynamic.dimension], 'predicted.covariance');

		return covariance;
	}

	/**
	This will return the new prediction, relatively to the dynamic model chosen
	* @param {State} previousCorrected State relative to our dynamic model
	* @returns{State} predicted State
	*/

	predict(options = {}) {
		let {previousCorrected, index} = options;
		previousCorrected = previousCorrected || this.getInitState();

		if (typeof (index) !== 'number' && typeof (previousCorrected.index) === 'number') {
			index = previousCorrected.index + 1;
		}

		State.check(previousCorrected, {dimension: this.dynamic.dimension});

		const getValueOptions = Object.assign({}, {
			previousCorrected,
			index
		}, options);
		const d = this.getValue(this.dynamic.transition, getValueOptions);

		checkMatrix(d, [this.dynamic.dimension, this.dynamic.dimension], 'dynamic.transition');

		const mean = matMul(d, previousCorrected.mean);

		const covariance = this.getPredictedCovariance(getValueOptions);

		const predicted = new State({mean, covariance, index});
		this.logger.debug('Prediction done', predicted);

		return predicted;
	}
	/**
	This will return the new correction, taking into account the prediction made
	and the observation of the sensor
	* @param {State} predicted the previous State
	* @returns{Array<Array>} kalmanGain
	*/

	getGain(options) {
		let {predicted, stateProjection} = options;
		const getValueOptions = Object.assign({}, {index: predicted.index}, options);
		stateProjection = stateProjection || this.getValue(this.observation.stateProjection, getValueOptions);
		const obsCovariance = this.getValue(this.observation.covariance, getValueOptions);
		checkMatrix(obsCovariance, [this.observation.dimension, this.observation.dimension], 'observation.covariance');

		const stateProjTransposed = transpose(stateProjection);
		const noiselessInnovation = matMul(
			matMul(stateProjection, predicted.covariance),
			stateProjTransposed
		);

		const innovationCovariance = add(noiselessInnovation, obsCovariance);

		const optimalKalmanGain = matMul(
			matMul(predicted.covariance, stateProjTransposed),
			invert(innovationCovariance)
		);

		return optimalKalmanGain;
	}

	/**
	This will return the corrected covariance of a given predicted State, this will help us to build the asymptoticState.
	* @param {State} predicted the previous State
	* @returns{Array.<Array.<Number>>}
	*/

	getCorrectedCovariance(options) {
		let {predicted, optimalKalmanGain, stateProjection} = options;
		const identity = getIdentity(predicted.covariance.length);
		if (!stateProjection) {
			const getValueOptions = Object.assign({}, {index: predicted.index}, options);
			stateProjection = this.getValue(this.observation.stateProjection, getValueOptions);
		}

		if (!optimalKalmanGain) {
			optimalKalmanGain = this.getGain(Object.assign({stateProjection}, options));
		}

		return matMul(
			sub(identity, matMul(optimalKalmanGain, stateProjection)),
			predicted.covariance
		);
	}

	/**
	This will return the new correction, taking into account the prediction made
	and the observation of the sensor
	* @param {State} predicted the previous State
	* @param {Array} observation the observation of the sensor
	* @returns{State} corrected State of the Kalman Filter
	*/

	correct(options) {
		const {predicted, observation} = options;
		State.check(predicted, {dimension: this.dynamic.dimension});
		if (!observation) {
			throw (new Error('no measure available'));
		}

		const getValueOptions = Object.assign({}, {observation, predicted, index: predicted.index}, options);
		const stateProjection = this.getValue(this.observation.stateProjection, getValueOptions);

		const optimalKalmanGain = this.getGain(Object.assign({predicted, stateProjection}, options));

		const innovation = sub(
			observation,
			matMul(stateProjection, predicted.mean)
		);
		const mean = add(
			predicted.mean,
			matMul(optimalKalmanGain, innovation)
		);
		if (Number.isNaN(mean[0][0])) {
			console.log({optimalKalmanGain, innovation, predicted});
			throw (new TypeError('Mean is NaN after correction'));
		}

		const covariance = this.getCorrectedCovariance(Object.assign({predicted, optimalKalmanGain, stateProjection}, options));
		const corrected = new State({mean, covariance, index: predicted.index});
		this.logger.debug('Correction done', corrected);
		return corrected;
	}
}

module.exports = CoreKalmanFilter;


/***/ }),

/***/ "./lib/dynamic/constant-acceleration.js":
/*!**********************************************!*\
  !*** ./lib/dynamic/constant-acceleration.js ***!
  \**********************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const identity = __webpack_require__(/*! ../linalgebra/identity.js */ "./lib/linalgebra/identity.js");

/**
*Creates a dynamic model, following constant acceleration model with respect with the dimensions provided in the observation parameters
* @param {DynamicConfig} dynamic
* @param {ObservationConfig} observation
* @returns {DynamicConfig}
*/

module.exports = function (dynamic, observation) {
	const timeStep = dynamic.timeStep || 1;
	const {observedProjection} = observation;
	const {stateProjection} = observation;
	const observationDimension = observation.dimension;
	let dimension;

	if (stateProjection && Number.isInteger(stateProjection[0].length / 3)) {
		dimension = observation.stateProjection[0].length;
	} else if (observedProjection) {
		dimension = observedProjection[0].length * 3;
	} else if (observationDimension) {
		dimension = observationDimension * 3;
	} else {
		throw (new Error('observedProjection or stateProjection should be defined in observation in order to use constant-speed filter'));
	}

	const baseDimension = dimension / 3;
	// We construct the transition and covariance matrices
	const transition = identity(dimension);
	for (let i = 0; i < baseDimension; i++) {
		transition[i][i + baseDimension] = timeStep;
		transition[i][i + (2 * baseDimension)] = 0.5 * (timeStep ** 2);
		transition[i + baseDimension][i + (2 * baseDimension)] = timeStep;
	}

	const arrayCovariance = new Array(baseDimension).fill(1)
		.concat(new Array(baseDimension).fill(timeStep * timeStep))
		.concat(new Array(baseDimension).fill(timeStep ** 4));
	const covariance = dynamic.covariance || arrayCovariance;
	return Object.assign({}, dynamic, {dimension, transition, covariance});
};


/***/ }),

/***/ "./lib/dynamic/constant-position.js":
/*!******************************************!*\
  !*** ./lib/dynamic/constant-position.js ***!
  \******************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const identity = __webpack_require__(/*! ../linalgebra/identity.js */ "./lib/linalgebra/identity.js");
/**
*Creates a dynamic model, following constant position model with respect with the dimensions provided in the observation parameters
* @param {DynamicConfig} dynamic
* @param {ObservationConfig} observation
* @returns {DynamicConfig}
*/

module.exports = function (dynamic, observation) {
	let {dimension} = dynamic;
	const observationDimension = observation.dimension;
	const {observedProjection} = observation;
	const {stateProjection} = observation;
	let {covariance} = dynamic;

	if (!dynamic.dimension) {
		if (observationDimension) {
			dimension = observationDimension;
		} else if (observedProjection) {
			dimension = observedProjection[0].length;
		} else if (stateProjection) {
			dimension = stateProjection[0].length;
		}
	}

	const transition = identity(dimension);
	covariance = covariance || identity(dimension);
	return Object.assign({}, dynamic, {dimension, transition, covariance});
};


/***/ }),

/***/ "./lib/dynamic/constant-speed.js":
/*!***************************************!*\
  !*** ./lib/dynamic/constant-speed.js ***!
  \***************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const identity = __webpack_require__(/*! ../linalgebra/identity.js */ "./lib/linalgebra/identity.js");

/**
*Creates a dynamic model, following constant position model with respect with the dimensions provided in the observation parameters
* @param {DynamicConfig} dynamic
* @param {ObservationConfig} observation
* @returns {DynamicConfig}
*/

module.exports = function (dynamic, observation) {
	const timeStep = dynamic.timeStep || 1;
	const {observedProjection} = observation;
	const {stateProjection} = observation;
	const observationDimension = observation.dimension;
	let dimension;

	if (stateProjection && Number.isInteger(stateProjection[0].length / 2)) {
		dimension = observation.stateProjection[0].length;
	} else if (observedProjection) {
		dimension = observedProjection[0].length * 2;
	} else if (observationDimension) {
		dimension = observationDimension * 2;
	} else {
		throw (new Error('observedProjection or stateProjection should be defined in observation in order to use constant-speed filter'));
	}

	const baseDimension = dimension / 2;
	// We construct the transition and covariance matrices
	const transition = identity(dimension);
	for (let i = 0; i < baseDimension; i++) {
		transition[i][i + baseDimension] = timeStep;
	}

	const arrayCovariance = new Array(baseDimension).fill(1).concat(new Array(baseDimension).fill(timeStep * timeStep));
	const covariance = dynamic.covariance || arrayCovariance;
	return Object.assign({}, dynamic, {dimension, transition, covariance});
};


/***/ }),

/***/ "./lib/kalman-filter.js":
/*!******************************!*\
  !*** ./lib/kalman-filter.js ***!
  \******************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {


const arrayToMatrix = __webpack_require__(/*! ../lib/utils/array-to-matrix.js */ "./lib/utils/array-to-matrix.js");
const setDimensions = __webpack_require__(/*! ../lib/setup/set-dimensions.js */ "./lib/setup/set-dimensions.js");
const checkDimensions = __webpack_require__(/*! ../lib/setup/check-dimensions.js */ "./lib/setup/check-dimensions.js");
const buildStateProjection = __webpack_require__(/*! ../lib/setup/build-state-projection.js */ "./lib/setup/build-state-projection.js");
const extendDynamicInit = __webpack_require__(/*! ../lib/setup/extend-dynamic-init.js */ "./lib/setup/extend-dynamic-init.js");
const toFunction = __webpack_require__(/*! ../lib/utils/to-function.js */ "./lib/utils/to-function.js");
const deepAssign = __webpack_require__(/*! ../lib/utils/deep-assign.js */ "./lib/utils/deep-assign.js");
const polymorphMatrix = __webpack_require__(/*! ../lib/utils/polymorph-matrix.js */ "./lib/utils/polymorph-matrix.js");
const distanceMat = __webpack_require__(/*! ../lib/linalgebra/distance-mat.js */ "./lib/linalgebra/distance-mat.js");
const State = __webpack_require__(/*! ./state.js */ "./lib/state.js");
const modelCollection = __webpack_require__(/*! ./model-collection.js */ "./lib/model-collection.js");
const CoreKalmanFilter = __webpack_require__(/*! ./core-kalman-filter.js */ "./lib/core-kalman-filter.js");

const buildDefaultDynamic = function (dynamic) {
	if (typeof (dynamic) === 'string') {
		return {name: dynamic};
	}

	return {name: 'constant-position'};
};

const buildDefaultObservation = function (observation) {
	if (typeof (observation) === 'number') {
		return {name: 'sensor', sensorDimension: observation};
	}

	if (typeof (observation) === 'string') {
		return {name: observation};
	}

	return {name: 'sensor'};
};
/**
*This function fills the given options by successively checking if it uses a registered model,
* it builds and checks the dynamic and observation dimensions, build the stateProjection if only observedProjection
*is given, and initialize dynamic.init
*@param {DynamicConfig} options.dynamic
*@param {ObservationConfig} options.observation
*/

const setupModelsParameters = function ({observation, dynamic}) {
	if (typeof (observation) !== 'object' || observation === null) {
		observation = buildDefaultObservation(observation);
	}

	if (typeof (dynamic) !== 'object' || dynamic === null) {
		dynamic = buildDefaultDynamic(dynamic, observation);
	}

	if (typeof (observation.name) === 'string') {
		observation = modelCollection.buildObservation(observation);
	}

	if (typeof (dynamic.name) === 'string') {
		dynamic = modelCollection.buildDynamic(dynamic, observation);
	}

	const withDimensionOptions = setDimensions({observation, dynamic});
	const checkedDimensionOptions = checkDimensions(withDimensionOptions);
	const buildStateProjectionOptions = buildStateProjection(checkedDimensionOptions);
	return extendDynamicInit(buildStateProjectionOptions);
};

/**
*Returns the corresponding model without arrays as values but only functions
*@param {ObservationConfig} observation
*@param {DynamicConfig} dynamic
*@returns {ObservationConfig, DynamicConfig} model with respect of the Core Kalman Filter properties
*/
const modelsParametersToCoreOptions = function (modelToBeChanged) {
	const {observation, dynamic} = modelToBeChanged;
	return deepAssign(modelToBeChanged, {
		observation: {
			stateProjection: toFunction(polymorphMatrix(observation.stateProjection)),
			covariance: toFunction(polymorphMatrix(observation.covariance, {dimension: observation.dimension}))
		},
		dynamic: {
			transition: toFunction(polymorphMatrix(dynamic.transition)),
			covariance: toFunction(polymorphMatrix(dynamic.covariance, {dimension: dynamic.dimension}))
		}
	});
};

class KalmanFilter extends CoreKalmanFilter {
	/**
	* @param {DynamicConfig} options.dynamic
	* @param {ObservationConfig} options.observation the system's observation model
	*/
	constructor(options = {}) {
		const modelsParameters = setupModelsParameters(options);
		const coreOptions = modelsParametersToCoreOptions(modelsParameters);

		super(Object.assign({}, options, coreOptions));
	}

	correct(options) {
		const coreObservation = arrayToMatrix({observation: options.observation, dimension: this.observation.dimension});
		return super.correct(Object.assign({}, options, {observation: coreObservation}));
	}

	/**
	*Performs the prediction and the correction steps
	*@param {State} previousCorrected
	*@param {<Array.<Number>>} observation
	*@returns {Array.<Number>} the mean of the corrections
	*/

	filter(options) {
		const predicted = super.predict(options);
		return this.correct(Object.assign({}, options, {predicted}));
	}

	/**
*Filters all the observations
*@param {Array.<Array.<Number>>} observations
*@returns {Array.<Array.<Number>>} the mean of the corrections
*/
	filterAll(observations) {
		const {mean: meanInit, covariance: covarianceInit, index: indexInit} = this.dynamic.init;
		let previousCorrected = new State({
			mean: meanInit,
			covariance: covarianceInit,
			index: indexInit});
		const results = [];
		for (const observation of observations) {
			const predicted = this.predict({previousCorrected});
			previousCorrected = this.correct({
				predicted,
				observation
			});
			results.push(previousCorrected.mean.map(m => m[0]));
		}

		return results;
	}

	/**
	* Returns an estimation of the asymptotic state covariance as explained in https://en.wikipedia.org/wiki/Kalman_filter#Asymptotic_form
	* in practice this can be used as a init.covariance value but is very costful calculation (that's why this is not made by default)
	* @param {Number} [tolerance=1e-6] returns when the last values differences are less than tolerance
	* @return {<Array.<Array.<Number>>>} covariance
	*/
	asymptoticStateCovariance(limitIterations = 1e2, tolerance = 1e-6) {
		let previousCorrected = super.getInitState();
		let predicted;
		const results = [];
		for (let i = 0; i < limitIterations; i++) {
			// We create a fake mean that will not be used in order to keep coherence
			predicted = new State({
				mean: null,
				covariance: super.getPredictedCovariance({previousCorrected})
			});
			previousCorrected = new State({
				mean: null,
				covariance: super.getCorrectedCovariance({predicted})
			});
			results.push(previousCorrected.covariance);
			if (distanceMat(previousCorrected.covariance, results[i - 1]) < tolerance) {
				return results[i];
			}
		}

		throw (new Error('The state covariance does not converge asymptotically'));
	}

	/**
	* Returns an estimation of the asymptotic gain, as explained in https://en.wikipedia.org/wiki/Kalman_filter#Asymptotic_form
	* @param {Number} [tolerance=1e-6] returns when the last values differences are less than tolerance
	* @return {<Array.<Array.<Number>>>} gain
	*/
	asymptoticGain(tolerance = 1e-6) {
		const covariance = this.asymptoticStateCovariance(tolerance);

		const asymptoticState = new State({
			// We create a fake mean that will not be used in order to keep coherence
			mean: new Array(covariance.length).fill(0).map(() => [0]),
			covariance
		});

		return super.getGain({previousCorrected: asymptoticState});
	}
}

module.exports = KalmanFilter;


/***/ }),

/***/ "./lib/linalgebra/add.js":
/*!*******************************!*\
  !*** ./lib/linalgebra/add.js ***!
  \*******************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const elemWise = __webpack_require__(/*! ./elem-wise */ "./lib/linalgebra/elem-wise.js");
/**
* Add matrixes together
* @param {...<Array.<Array.<Number>>} args list of matrix
* @returns {Array.<Array.<Number>>} sum
*/
module.exports = function (...args) {
	return elemWise(args, args2 => {
		return args2.reduce((a, b) => {
			if (a === null || b === null) {
				return null;
			}

			return a + b;
		}, 0);
	});
};


/***/ }),

/***/ "./lib/linalgebra/diag.js":
/*!********************************!*\
  !*** ./lib/linalgebra/diag.js ***!
  \********************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const zeros = __webpack_require__(/*! ./zeros */ "./lib/linalgebra/zeros.js");

module.exports = function (mat) {
	const result = zeros(mat.length, mat.length);

	for (const [i, element] of mat.entries()) {
		result[i][i] = element;
	}

	return result;
};


/***/ }),

/***/ "./lib/linalgebra/distance-mat.js":
/*!****************************************!*\
  !*** ./lib/linalgebra/distance-mat.js ***!
  \****************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const trace = __webpack_require__(/*! ./trace.js */ "./lib/linalgebra/trace.js");
const transpose = __webpack_require__(/*! ./transpose.js */ "./lib/linalgebra/transpose.js");
const matSub = __webpack_require__(/*! ./sub.js */ "./lib/linalgebra/sub.js");
const matMul = __webpack_require__(/*! ./mat-mul.js */ "./lib/linalgebra/mat-mul.js");
const sum = __webpack_require__(/*! ./sum.js */ "./lib/linalgebra/sum.js");

// [Frobenius norm](https://en.wikipedia.org/wiki/Matrix_norm#Frobenius_norm )
module.exports = function (array1, array2) {
	if (typeof (array1) === 'undefined') {
		return sum(array2);
	}

	if (typeof (array2) === 'undefined') {
		return sum(array1);
	}

	const m = matSub(array1, array2);
	const p = matMul(transpose(m), m);
	return Math.sqrt(trace(p));
};


/***/ }),

/***/ "./lib/linalgebra/elem-wise.js":
/*!*************************************!*\
  !*** ./lib/linalgebra/elem-wise.js ***!
  \*************************************/
/***/ ((module) => {

/**
* @callback elemWiseCb
* @param {Array.<Number>} arr
* @param {Number} rowId
* @param {Number} colId
*/
/**
* run a function on cell per cell for each Matrixes
* @param {<Array.<Array.<Array.<Number>>>} arrMatrixes list of matrixes
* @param {elemWiseCb} fn
* @returns {Array.<Array.<Number>>} resulting matrix
* @example
// this will do m1 + m2 + m3 + m4 on matrixes
elemWise([m1, m2, m3, m4], args2 => {
	return args2.reduce((a, b) => a + b, 0);
});
*/

module.exports = function (arrayMatrixes, fn) {
	return arrayMatrixes[0].map((row, rowId) => {
		return row.map((cell, colId) => {
			const array = arrayMatrixes.map(m => m[rowId][colId]);
			return fn(array, rowId, colId);
		});
	});
};



/***/ }),

/***/ "./lib/linalgebra/identity.js":
/*!************************************!*\
  !*** ./lib/linalgebra/identity.js ***!
  \************************************/
/***/ ((module) => {

module.exports = function (stateSize) {
	const identityArray = [];
	for (let i = 0; i < stateSize; i++) {
		const rowIdentity = [];
		for (let j = 0; j < stateSize; j++) {
			if (i === j) {
				rowIdentity.push(1);
			} else {
				rowIdentity.push(0);
			}
		}

		identityArray.push(rowIdentity);
	}

	return identityArray;
};


/***/ }),

/***/ "./lib/linalgebra/index.js":
/*!*********************************!*\
  !*** ./lib/linalgebra/index.js ***!
  \*********************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

module.exports = {
	add: __webpack_require__(/*! ./add.js */ "./lib/linalgebra/add.js"),
	diag: __webpack_require__(/*! ./diag.js */ "./lib/linalgebra/diag.js"),
	'distance-mat': __webpack_require__(/*! ./distance-mat.js */ "./lib/linalgebra/distance-mat.js"),
	'elem-wise': __webpack_require__(/*! ./elem-wise.js */ "./lib/linalgebra/elem-wise.js"),
	identity: __webpack_require__(/*! ./identity.js */ "./lib/linalgebra/identity.js"),
	invert: __webpack_require__(/*! ./invert.js */ "./lib/linalgebra/invert.js"),
	'mat-mul': __webpack_require__(/*! ./mat-mul.js */ "./lib/linalgebra/mat-mul.js"),
	'pad-with-zeros': __webpack_require__(/*! ./pad-with-zeros.js */ "./lib/linalgebra/pad-with-zeros.js"),
	sub: __webpack_require__(/*! ./sub.js */ "./lib/linalgebra/sub.js"),
	'sub-square-matrix': __webpack_require__(/*! ./sub-square-matrix.js */ "./lib/linalgebra/sub-square-matrix.js"),
	sum: __webpack_require__(/*! ./sum.js */ "./lib/linalgebra/sum.js"),
	trace: __webpack_require__(/*! ./trace.js */ "./lib/linalgebra/trace.js"),
	transpose: __webpack_require__(/*! ./transpose.js */ "./lib/linalgebra/transpose.js"),
	zeros: __webpack_require__(/*! ./zeros.js */ "./lib/linalgebra/zeros.js")
};


/***/ }),

/***/ "./lib/linalgebra/invert.js":
/*!**********************************!*\
  !*** ./lib/linalgebra/invert.js ***!
  \**********************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const matrixInverse = __webpack_require__(/*! matrix-inverse */ "./node_modules/matrix-inverse/matrix-inverse.js");

module.exports = function (m) {
	return matrixInverse(m);
};


/***/ }),

/***/ "./lib/linalgebra/mat-mul.js":
/*!***********************************!*\
  !*** ./lib/linalgebra/mat-mul.js ***!
  \***********************************/
/***/ ((module) => {

/**
* Multiply 2 matrixes together
* @param {<Array.<Array.<Number>>} m1
* @param {<Array.<Array.<Number>>} m2
* @returns {Array.<Array.<Number>>}
*/
module.exports = function (m1, m2) {
	// Console.log({m1, m2});
	const result = [];
	for (let i = 0; i < m1.length; i++) {
		result[i] = [];
		for (let j = 0; j < m2[0].length; j++) {
			let sum = 0;
			let isNull = false;
			for (let k = 0; k < m1[0].length; k++) {
				if ((m1[i][k] === null && m2[k][j] !== 0) || (m2[k][j] === null && m1[i][k] !== 0)) {
					isNull = true;
				}

				sum += m1[i][k] * m2[k][j];
			}

			result[i][j] = isNull ? null : sum;
		}
	}

	return result;
};


/***/ }),

/***/ "./lib/linalgebra/pad-with-zeros.js":
/*!******************************************!*\
  !*** ./lib/linalgebra/pad-with-zeros.js ***!
  \******************************************/
/***/ ((module) => {

/**
*This function returns the stateProjection paded with zeros with respect to a given
*observedProjection
*@param {Array.<Number> | Array.<Array.<Number>>} array the array we need to pad
*@param {Number} dimension in our case, the dynamic dimension
*@returns {Array.<Number> | Array.<Array.<Number>>} paded array
*/
module.exports = function (array, {dimension}) {
	const l1 = array.length;
	const l = array[0].length;
	const result = array.map(a => a.concat());

	if (dimension < l) {
		throw (new TypeError(`Dynamic dimension ${dimension} does not match with observedProjection ${l}`));
	}

	for (let i = 0; i < l1; i++) {
		for (let j = 0; j < dimension - l; j++) {
			result[i].push(0);
		}
	}

	return result;
};


/***/ }),

/***/ "./lib/linalgebra/sub-square-matrix.js":
/*!*********************************************!*\
  !*** ./lib/linalgebra/sub-square-matrix.js ***!
  \*********************************************/
/***/ ((module) => {

module.exports = (mat, obsIndexes) => {
	return obsIndexes.map(s1 => obsIndexes.map(s2 => mat[s1][s2]));
};


/***/ }),

/***/ "./lib/linalgebra/sub.js":
/*!*******************************!*\
  !*** ./lib/linalgebra/sub.js ***!
  \*******************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const elemWise = __webpack_require__(/*! ./elem-wise */ "./lib/linalgebra/elem-wise.js");

module.exports = function (...args) {
	return elemWise(args, ([a, b]) => a - b);
};


/***/ }),

/***/ "./lib/linalgebra/sum.js":
/*!*******************************!*\
  !*** ./lib/linalgebra/sum.js ***!
  \*******************************/
/***/ ((module) => {

// Sum all the terms of a given matrix
module.exports = function (array) {
	let s = 0;
	for (let i = 0; i < array.length; i++) {
		for (let j = 0; j < array.length; j++) {
			s += array[i][j];
		}
	}

	return s;
};


/***/ }),

/***/ "./lib/linalgebra/trace.js":
/*!*********************************!*\
  !*** ./lib/linalgebra/trace.js ***!
  \*********************************/
/***/ ((module) => {

module.exports = function (array) {
	let diag = 0;
	for (const [row, element] of array.entries()) {
		diag += element[row];
	}

	return diag;
};


/***/ }),

/***/ "./lib/linalgebra/transpose.js":
/*!*************************************!*\
  !*** ./lib/linalgebra/transpose.js ***!
  \*************************************/
/***/ ((module) => {

module.exports = function (array) {
	return array[0].map((col, i) => array.map(row => row[i]));
};


/***/ }),

/***/ "./lib/linalgebra/zeros.js":
/*!*********************************!*\
  !*** ./lib/linalgebra/zeros.js ***!
  \*********************************/
/***/ ((module) => {

module.exports = function (rows, cols) {
	return new Array(rows).fill(1).map(() => new Array(cols).fill(0));
};


/***/ }),

/***/ "./lib/model-collection.js":
/*!*********************************!*\
  !*** ./lib/model-collection.js ***!
  \*********************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const registeredDynamicModels = {
	'constant-position': __webpack_require__(/*! ../lib/dynamic/constant-position.js */ "./lib/dynamic/constant-position.js"),
	'constant-speed': __webpack_require__(/*! ../lib/dynamic/constant-speed.js */ "./lib/dynamic/constant-speed.js"),
	'constant-acceleration': __webpack_require__(/*! ../lib/dynamic/constant-acceleration.js */ "./lib/dynamic/constant-acceleration.js")
};
const registeredObservationModels = {
	sensor: __webpack_require__(/*! ../lib/observation/sensor.js */ "./lib/observation/sensor.js")
};

/**
*RegisterObservation enables to create a new observation model and stock it
* @param {String} name
* @callback fn the function corresponding to the desired model
*/

/**
*registerDynamic enables to create a new dynamic model and stocks it
* @param {String} name
* @callback fn the function corresponding to the desired model
*/

/**
*buildObservation enables to build a model given an observation configuration
* @param {ObservationConfig} observation
* @returns {ObservationConfig} the configuration with respect to the model
*/

/**
*buildDynamic enables to build a model given dynamic and observation configurations
* @param {DynamicConfig} dynamic
* @param {ObservationConfig} observation
* @returns {DynamicConfig} the dynamic configuration with respect to the model
*/

module.exports = {
	registerObservation: (name, fn) => {
		registeredObservationModels[name] = fn;
	},
	registerDynamic: (name, fn) => {
		registeredDynamicModels[name] = fn;
	},
	buildObservation: observation => {
		if (!registeredObservationModels[observation.name]) {
			throw (new Error(`The provided observation model name (${observation.name}) is not registered`));
		}

		return registeredObservationModels[observation.name](observation);
	},
	buildDynamic: (dynamic, observation) => {
		if (!registeredDynamicModels[dynamic.name]) {
			throw (new Error(`The provided dynamic model (${dynamic.name}) name is not registered`));
		}

		return registeredDynamicModels[dynamic.name](dynamic, observation);
	}
};


/***/ }),

/***/ "./lib/observation/sensor.js":
/*!***********************************!*\
  !*** ./lib/observation/sensor.js ***!
  \***********************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const identity = __webpack_require__(/*! ../linalgebra/identity.js */ "./lib/linalgebra/identity.js");
const polymorphMatrix = __webpack_require__(/*! ../utils/polymorph-matrix.js */ "./lib/utils/polymorph-matrix.js");
const checkMatrix = __webpack_require__(/*! ../utils/check-matrix.js */ "./lib/utils/check-matrix.js");

/**
* @param {Number} sensorDimension
* @param {CovarianceParam} sensorCovariance
* @param {Number} nSensors
* @returns {ObservationConfig}
*/

const copy = mat => mat.map(a => a.concat());

module.exports = function (options) {
	const {sensorDimension = 1, sensorCovariance = 1, nSensors = 1} = options;
	const sensorCovarianceFormatted = polymorphMatrix(sensorCovariance, {dimension: sensorDimension});
	checkMatrix(sensorCovarianceFormatted, [sensorDimension, sensorDimension], 'observation.sensorCovariance');
	const oneSensorObservedProjection = identity(sensorDimension);
	let concatenatedObservedProjection = [];
	const dimension = sensorDimension * nSensors;
	const concatenatedCovariance = identity(dimension);
	for (let i = 0; i < nSensors; i++) {
		concatenatedObservedProjection = concatenatedObservedProjection.concat(copy(oneSensorObservedProjection));

		sensorCovarianceFormatted.forEach((r, rIndex) => r.forEach((c, cIndex) => {
			concatenatedCovariance[rIndex + (i * sensorDimension)][cIndex + (i * sensorDimension)] = c;
		}));
	}

	return Object.assign({}, options, {
		dimension,
		observedProjection: concatenatedObservedProjection,
		covariance: concatenatedCovariance
	});
};


/***/ }),

/***/ "./lib/setup/build-state-projection.js":
/*!*********************************************!*\
  !*** ./lib/setup/build-state-projection.js ***!
  \*********************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const padWithZeros = __webpack_require__(/*! ../linalgebra/pad-with-zeros.js */ "./lib/linalgebra/pad-with-zeros.js");
const identity = __webpack_require__(/*! ../linalgebra/identity.js */ "./lib/linalgebra/identity.js");
/**
*Builds the stateProjection given an observedProjection
*@param {ObservationConfig} observation
*@param {DynamicConfig} dynamic
*@returns {ObservationConfig, DynamicConfig} the model containing the created stateProjection
*/

module.exports = function ({observation, dynamic}) {
	const {observedProjection, stateProjection} = observation;
	const observationDimension = observation.dimension;
	const dynamicDimension = dynamic.dimension;
	if (observedProjection && stateProjection) {
		throw (new TypeError('You cannot use both observedProjection and stateProjection'));
	}

	if (observedProjection) {
		const stateProjection = padWithZeros(observedProjection, {dimension: dynamicDimension});
		return {
			observation: Object.assign({}, observation, {
				stateProjection
			}),
			dynamic
		};
	}

	if (observationDimension && dynamicDimension && !stateProjection) {
		const observationMatrix = identity(observationDimension);
		return {
			observation: Object.assign({}, observation, {
				stateProjection: padWithZeros(observationMatrix, {dimension: dynamicDimension})
			}),
			dynamic
		};
	}

	return {observation, dynamic};
};


/***/ }),

/***/ "./lib/setup/check-dimensions.js":
/*!***************************************!*\
  !*** ./lib/setup/check-dimensions.js ***!
  \***************************************/
/***/ ((module) => {

/**
*Verifies that dynamic.dimension and observation.dimension are set
*@param {ObservationConfig} observation
*@param {DynamicConfig} dynamic
*/

module.exports = function ({observation, dynamic}) {
	const dynamicDimension = dynamic.dimension;
	const observationDimension = observation.dimension;
	if (!dynamicDimension || !observationDimension) {
		throw (new TypeError('Dimension is not set'));
	}

	return {observation, dynamic};
};


/***/ }),

/***/ "./lib/setup/extend-dynamic-init.js":
/*!******************************************!*\
  !*** ./lib/setup/extend-dynamic-init.js ***!
  \******************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const diag = __webpack_require__(/*! ../linalgebra/diag.js */ "./lib/linalgebra/diag.js");

/**
*Initializes the dynamic.init when not given
*@param {ObservationConfig} observation
*@param {DynamicConfig} dynamic
*@returns {ObservationConfig, DynamicConfig}
*/

module.exports = function ({observation, dynamic}) {
	if (!dynamic.init) {
		const huge = 1e6;
		const dynamicDimension = dynamic.dimension;
		const meanArray = new Array(dynamicDimension).fill(0);
		const covarianceArray = new Array(dynamicDimension).fill(huge);
		const withInitOptions = {
			observation,
			dynamic: Object.assign({}, dynamic, {
				init: {
					mean: meanArray.map(element => [element]),
					covariance: diag(covarianceArray),
					index: -1
				}
			})
		};
		return withInitOptions;
	}

	return {observation, dynamic};
};


/***/ }),

/***/ "./lib/setup/set-dimensions.js":
/*!*************************************!*\
  !*** ./lib/setup/set-dimensions.js ***!
  \*************************************/
/***/ ((module) => {

/**
*Verifies that dimensions are matching and set dynamic.dimension and observation.dimension
* with respect of stateProjection and transition dimensions
*@param {ObservationConfig} observation
*@param {DynamicConfig} dynamic
*@returns {ObservationConfig, DynamicConfig}
*/

module.exports = function ({observation, dynamic}) {
	const {stateProjection} = observation;
	const {transition} = dynamic;
	const dynamicDimension = dynamic.dimension;
	const observationDimension = observation.dimension;

	if (dynamicDimension && observationDimension && Array.isArray(stateProjection) && (dynamicDimension !== stateProjection[0].length || observationDimension !== stateProjection.length)) {
		throw (new TypeError('stateProjection dimensions not matching with observation and dynamic dimensions'));
	}

	if (dynamicDimension && Array.isArray(transition) && dynamicDimension !== transition.length) {
		throw (new TypeError('transition dimension not matching with dynamic dimension'));
	}

	if (Array.isArray(stateProjection)) {
		return {
			observation: Object.assign({}, observation, {
				dimension: stateProjection.length
			}),
			dynamic: Object.assign({}, dynamic, {
				dimension: stateProjection[0].length
			})
		};
	}

	if (Array.isArray(transition)) {
		return {
			observation,
			dynamic: Object.assign({}, dynamic, {
				dimension: transition.length
			})
		};
	}

	return {observation, dynamic};
};


/***/ }),

/***/ "./lib/state.js":
/*!**********************!*\
  !*** ./lib/state.js ***!
  \**********************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const sub = __webpack_require__(/*! ./linalgebra/sub.js */ "./lib/linalgebra/sub.js");
const transpose = __webpack_require__(/*! ./linalgebra/transpose.js */ "./lib/linalgebra/transpose.js");
const matMul = __webpack_require__(/*! ./linalgebra/mat-mul.js */ "./lib/linalgebra/mat-mul.js");
const invert = __webpack_require__(/*! ./linalgebra/invert.js */ "./lib/linalgebra/invert.js");
const elemWise = __webpack_require__(/*! ./linalgebra/elem-wise.js */ "./lib/linalgebra/elem-wise.js");
const subSquareMatrix = __webpack_require__(/*! ./linalgebra/sub-square-matrix */ "./lib/linalgebra/sub-square-matrix.js");
const arrayToMatrix = __webpack_require__(/*! ./utils/array-to-matrix.js */ "./lib/utils/array-to-matrix.js");

const checkMatrix = __webpack_require__(/*! ./utils/check-matrix.js */ "./lib/utils/check-matrix.js");
const checkCovariance = __webpack_require__(/*! ./utils/check-covariance */ "./lib/utils/check-covariance.js");

/**
 * @class
 * Class representing a multi dimensionnal gaussian, with his mean and his covariance
 * @property {Number} [index=0] the index of the State in the process, this is not mandatory for simple Kalman Filter, but is needed for most of the use case of extended kalman filter
 * @property {Array.<Array.<Number>>} covariance square matrix of size dimension
 * @property {Array.<Array<Number>>} mean column matrix of size dimension x 1
 */
class State {
	constructor({mean, covariance, index}) {
		this.mean = mean;
		this.covariance = covariance;
		this.index = index;
	}

	/**
	* Check the consistency of the State
	*/
	check(options) {
		this.constructor.check(this, options);
	}

	/**
	* Check the consistency of the State's attributes
	*/

	static check(state, {dimension = null, title = null, eigen} = {}) {
		if (!(state instanceof State)) {
			throw (new TypeError(
				'The argument is not a state \n' +
        'Tips: maybe you are using 2 different version of kalman-filter in your npm deps tree'
			));
		}

		const {mean, covariance} = state; // Index
		const meanDimension = mean.length;
		if (typeof (dimension) === 'number' && meanDimension !== dimension) {
			throw (new Error(`[${title}] State.mean ${mean} with dimension ${meanDimension} does not match expected dimension (${dimension})`));
		}

		checkMatrix(mean, [meanDimension, 1], title ? title + '-mean' : 'mean');
		checkMatrix(covariance, [meanDimension, meanDimension], title ? title + '-covariance' : 'covariance');
		checkCovariance({covariance, eigen}, title ? title + '-covariance' : 'covariance');
		// If (typeof (index) !== 'number') {
		// 	throw (new TypeError('t must be a number'));
		// }
	}

	static matMul({state, matrix}) {
		const covariance = matMul(
			matMul(matrix, state.covariance),
			transpose(matrix)
		);
		const mean = matMul(matrix, state.mean);

		return new State({
			mean,
			covariance,
			index: state.index
		});
	}

	/**
	* From a state in n-dimension create a state in a subspace
	* If you see the state as a N-dimension gaussian,
	* this can be viewed as the sub M-dimension gaussian (M < N)
	* @param {Array.<Number>} obsIndexes list of dimension to extract,  (M < N <=> obsIndexes.length < this.mean.length)
	* @returns {State} subState in subspace, with subState.mean.length === obsIndexes.length
	*/
	subState(obsIndexes) {
		const state = new State({
			mean: obsIndexes.map(i => this.mean[i]),
			covariance: subSquareMatrix(this.covariance, obsIndexes),
			index: this.index
		});
		return state;
	}

	/**
	* Simple Malahanobis distance between the distribution (this) and a point
	* @param {Array.<[Number]>} point a Nx1 matrix representing a point
	*/
	rawDetailedMahalanobis(point) {
		const diff = sub(this.mean, point);
		this.check();
		const covarianceInvert = invert(this.covariance);
		if (covarianceInvert === null) {
			this.check({eigen: true});
			throw (new Error(`Cannot invert covariance ${JSON.stringify(this.covariance)}`));
		}

		const diffTransposed = transpose(diff);

		// Console.log('covariance in obs space', covarianceInObservationSpace);

		const value = Math.sqrt(
			matMul(
				matMul(
					diffTransposed,
					covarianceInvert
				),
				diff
			)
		);
		if (Number.isNaN(value)) {
			console.log({diff, covarianceInvert, this: this, point}, matMul(
				matMul(
					diffTransposed,
					covarianceInvert
				),
				diff
			));
			throw (new Error('mahalanobis is NaN'));
		}

		return {
			diff,
			covarianceInvert,
			value
		};
	}

	/**
	* Malahanobis distance is made against an observation, so the mean and covariance
	* are projected into the observation space
	* @param {KalmanFilter} kf kalman filter use to project the state in observation's space
	* @param {Observation} observation
	* @param {Array.<Number>} obsIndexes list of indexes of observation state to use for the mahalanobis distance
	*/
	detailedMahalanobis({kf, observation, obsIndexes}) {
		if (observation.length !== kf.observation.dimension) {
			throw (new Error(`Mahalanobis observation ${observation} (dimension: ${observation.length}) does not match with kf observation dimension (${kf.observation.dimension})`));
		}

		let correctlySizedObservation = arrayToMatrix({observation, dimension: observation.length});

		const stateProjection = kf.getValue(kf.observation.stateProjection, {});

		let projectedState = this.constructor.matMul({state: this, matrix: stateProjection});

		if (Array.isArray(obsIndexes)) {
			projectedState = projectedState.subState(obsIndexes);
			correctlySizedObservation = obsIndexes.map(i => correctlySizedObservation[i]);
		}

		return projectedState.rawDetailedMahalanobis(correctlySizedObservation);
	}

	/**
	* @param {KalmanFilter} kf kalman filter use to project the state in observation's space
	* @param {Observation} observation
	* @param {Array.<Number>} obsIndexes list of indexes of observation state to use for the mahalanobis distance
	* @returns {Number}
	*/
	mahalanobis(options) {
		const result = this.detailedMahalanobis(options).value;
		if (Number.isNaN(result)) {
			throw (new TypeError('mahalanobis is NaN'));
		}

		return result;
	}

	/**
	* Bhattacharyya distance is made against in the observation space
	* to do it in the normal space see state.bhattacharyya
	* @param {KalmanFilter} kf kalman filter use to project the state in observation's space
	* @param {State} state
	* @param {Array.<Number>} obsIndexes list of indexes of observation state to use for the bhattacharyya distance
	* @returns {Number}
	*/
	obsBhattacharyya({kf, state, obsIndexes}) {
		const stateProjection = kf.getValue(kf.observation.stateProjection, {});

		let projectedSelfState = this.constructor.matMul({state: this, matrix: stateProjection});
		let projectedOtherState = this.constructor.matMul({state, matrix: stateProjection});

		if (Array.isArray(obsIndexes)) {
			projectedSelfState = projectedSelfState.subState(obsIndexes);
			projectedOtherState = projectedOtherState.subState(obsIndexes);
		}

		return projectedSelfState.bhattacharyya(projectedOtherState);
	}

	/**
	* @param {State} otherState other state to compare with
	* @returns {Number}
	*/
	bhattacharyya(otherState) {
		const state = this;
		const average = elemWise([state.covariance, otherState.covariance], ([a, b]) => (a + b) / 2);

		let covarInverted;
		try {
			covarInverted = invert(average);
		} catch (error) {
			console.log('Cannot invert', average);
			throw (error);
		}

		const diff = sub(state.mean, otherState.mean);

		return matMul(transpose(diff), matMul(covarInverted, diff))[0][0];
	}
}

module.exports = State;


/***/ }),

/***/ "./lib/utils/array-to-matrix.js":
/*!**************************************!*\
  !*** ./lib/utils/array-to-matrix.js ***!
  \**************************************/
/***/ ((module) => {

/**
*Returns the corresponding matrix in dim*1, given an dim matrix, and checks
* if corresponding with the observation dimension
*@param {Array.<Number> | Array.<Array.<Number>>} observation
*@param {Number} dimension
*@returns {Array.<Array.<Number>>}
*/

module.exports = function ({observation, dimension}) {
	if (!Array.isArray(observation)) {
		if (dimension === 1 && typeof (observation) === 'number') {
			return [[observation]];
		}

		throw (new TypeError(`The observation (${observation}) should be an array (dimension: ${dimension})`));
	}

	if (observation.length !== dimension) {
		throw (new TypeError(`Observation (${observation.length}) and dimension (${dimension}) not matching`));
	}

	if (typeof (observation[0]) === 'number' || observation[0] === null) {
		return observation.map(element => [element]);
	}

	return observation;
};


/***/ }),

/***/ "./lib/utils/check-covariance.js":
/*!***************************************!*\
  !*** ./lib/utils/check-covariance.js ***!
  \***************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const tolerance = 0.1;
const Matrix = __webpack_require__(/*! @rayyamhk/matrix */ "./node_modules/@rayyamhk/matrix/lib/index.js");
const checkMatrix = __webpack_require__(/*! ./check-matrix */ "./lib/utils/check-matrix.js");

const checkDefinitePositive = function (covariance, tolerance = 1e-10) {
	const covarianceMatrix = new Matrix(covariance);
	const eigenvalues = covarianceMatrix.eigenvalues();
	eigenvalues.forEach(eigenvalue => {
		if (eigenvalue <= -tolerance) {
			console.log(covariance, eigenvalue);
			throw new Error(`Eigenvalue should be positive (actual: ${eigenvalue})`);
		}
	});
	console.log('is definite positive', covariance);
};

const checkSymetric = function (covariance, title = 'checkSymetric') {
	covariance.forEach((row, rowId) => row.forEach((item, colId) => {
		if (rowId === colId && item < 0) {
			throw new Error(`[${title}] Variance[${colId}] should be positive (actual: ${item})`);
		} else if (Math.abs(item) > Math.sqrt(covariance[rowId][rowId] * covariance[colId][colId])) {
			console.log(covariance);
			throw new Error(`[${title}] Covariance[${rowId}][${colId}] should verify Cauchy Schwarz Inequality ` +
				`(expected: |x| <= sqrt(${covariance[rowId][rowId]} * ${covariance[colId][colId]})` +
				` actual: ${item})`);
		} else if (Math.abs(item - covariance[colId][rowId]) > tolerance) {
			throw new Error(`[${title}] Covariance[${rowId}][${colId}] should equal Covariance[${colId}][${rowId}] ` +
			` (actual diff: ${Math.abs(item - covariance[colId][rowId])})  = ${item} - ${covariance[colId][rowId]}\n` +
			`${covariance.join('\n')} is invalid`
			);
		}
	}));
};

module.exports = function ({covariance, eigen = false}) {
	checkMatrix(covariance);
	checkSymetric(covariance);
	if (eigen) {
		checkDefinitePositive(covariance);
	}
};


/***/ }),

/***/ "./lib/utils/check-matrix.js":
/*!***********************************!*\
  !*** ./lib/utils/check-matrix.js ***!
  \***********************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const checkShape = __webpack_require__(/*! ./check-shape */ "./lib/utils/check-shape.js");

module.exports = function (matrix, shape, title = 'checkMatrix') {
	if (matrix.reduce((a, b) => a.concat(b)).filter(a => Number.isNaN(a)).length > 0) {
		throw (new Error(
			`[${title}] Matrix should not have a NaN\nIn : \n` +
			matrix.join('\n')
		));
	}

	if (shape) {
		checkShape(matrix, shape, title);
	}
};


/***/ }),

/***/ "./lib/utils/check-shape.js":
/*!**********************************!*\
  !*** ./lib/utils/check-shape.js ***!
  \**********************************/
/***/ ((module) => {

const checkShape = function (matrix, shape, title = 'checkShape') {
	if (matrix.length !== shape[0]) {
		throw (new Error(`[${title}] expected size (${shape[0]}) and length (${matrix.length}) does not match`));
	}

	if (shape.length > 1) {
		return matrix.forEach(m => checkShape(m, shape.slice(1), title));
	}
};

module.exports = checkShape;


/***/ }),

/***/ "./lib/utils/correlation-to-covariance.js":
/*!************************************************!*\
  !*** ./lib/utils/correlation-to-covariance.js ***!
  \************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const checkCovariance = __webpack_require__(/*! ./check-covariance */ "./lib/utils/check-covariance.js");

module.exports = function ({correlation, variance}) {
	checkCovariance({covariance: correlation});
	return correlation.map((c, rowIndex) => c.map((a, colIndex) => a * Math.sqrt(variance[colIndex] * variance[rowIndex])));
};


/***/ }),

/***/ "./lib/utils/covariance-to-correlation.js":
/*!************************************************!*\
  !*** ./lib/utils/covariance-to-correlation.js ***!
  \************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const checkCovariance = __webpack_require__(/*! ./check-covariance */ "./lib/utils/check-covariance.js");

module.exports = function (covariance) {
	checkCovariance({covariance});
	const variance = covariance.map((_, i) => covariance[i][i]);

	return {
		variance,
		correlation: covariance.map((c, rowIndex) => c.map((a, colIndex) => a / Math.sqrt(variance[colIndex] * variance[rowIndex])))
	};
};


/***/ }),

/***/ "./lib/utils/deep-assign.js":
/*!**********************************!*\
  !*** ./lib/utils/deep-assign.js ***!
  \**********************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const uniq = __webpack_require__(/*! ./uniq.js */ "./lib/utils/uniq.js");

const limit = 100;

/**
*Equivalent to the Object.assign methode, takes several arguments and creates a new object corresponding to the assignment of the arguments
* @param {Object} args
* @param {Number} step
*/
const deepAssign = function (args, step) {
	if (step > limit) {
		throw (new Error(`In deepAssign, number of recursive call (${step}) reached limit (${limit}), deepAssign is not working on  self-referencing objects`));
	}

	const filterArguments = args.filter(arg => typeof (arg) !== 'undefined' && arg !== null);
	const lastArgument = filterArguments[filterArguments.length - 1];
	if (filterArguments.length === 1) {
		return filterArguments[0];
	}

	if (typeof (lastArgument) !== 'object' || Array.isArray(lastArgument)) {
		return lastArgument;
	}

	if (filterArguments.length === 0) {
		return null;
	}

	const objectsArguments = filterArguments.filter(arg => typeof (arg) === 'object');
	let keys = [];
	objectsArguments.forEach(arg => {
		keys = keys.concat(Object.keys(arg));
	});
	const uniqKeys = uniq(keys);
	const result = {};
	uniqKeys.forEach(key => {
		const values = objectsArguments.map(arg => arg[key]);
		result[key] = deepAssign(values, step + 1);
	});
	return result;
};

module.exports = ((...args) => deepAssign(args, 0));


/***/ }),

/***/ "./lib/utils/get-covariance.js":
/*!*************************************!*\
  !*** ./lib/utils/get-covariance.js ***!
  \*************************************/
/***/ ((module) => {

/**
* @param {Object} opts
* @param {Array.<Array.<Number>>} opts.measures a list of measure, size is LxN L the number of sample, N the dimension
* @param {Array.<Array.<Number>>} opts.averages a list of averages, size is LxN L the number of sample, N the dimension
* @returns {Array.<Array.<Number>>} covariance matrix size is NxN
*/

module.exports = function ({measures, averages}) {
	const l = measures.length;
	const n = measures[0].length;

	if (l === 0) {
		throw (new Error('Cannot find covariance for empty sample'));
	}

	return (new Array(n).fill(1)).map((_, rowIndex) => {
		return (new Array(n).fill(1)).map((_, colIndex) => {
			const stds = measures.map((m, i) => (m[rowIndex] - averages[i][rowIndex]) * (m[colIndex] - averages[i][colIndex]));
			const result = stds.reduce((a, b) => a + b) / l;
			if (Number.isNaN(result)) {
				throw (new TypeError('result is NaN'));
			}

			return result;
		});
	});
};


/***/ }),

/***/ "./lib/utils/polymorph-matrix.js":
/*!***************************************!*\
  !*** ./lib/utils/polymorph-matrix.js ***!
  \***************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

/**
* @typedef {Number | Array.<Number> | Array.<Array.<Number>>} CovarianceParam
*/
const diag = __webpack_require__(/*! ../linalgebra/diag */ "./lib/linalgebra/diag.js");
const checkMatrix = __webpack_require__(/*! ./check-matrix */ "./lib/utils/check-matrix.js");
/**
* If cov is a number, result will be Identity*cov
* If cov is an Array.<Number>, result will be diag(cov)
* If cov is an Array.<Array.<Number>>, result will be cov
* @param {CovarianceParam} cov
* @param {Number} dimension
* @returns {Array.<Array.<Number>>}
*/
module.exports = function (array, {dimension, title = 'polymorph'} = {}) {
	if (typeof (array) === 'number' || Array.isArray(array)) {
		if (typeof (array) === 'number' && typeof (dimension) === 'number') {
			return diag(new Array(dimension).fill(array));
		}

		if ((Array.isArray(array)) && (Array.isArray(array[0]))) {
			let shape;
			if (typeof (dimension) === 'number') {
				shape = [dimension, dimension];
			}

			checkMatrix(array, shape, title);
			return array;
		}

		if ((Array.isArray(array)) && (typeof (array[0]) === 'number')) {
			return diag(array);
		}
	}

	return array;
};


/***/ }),

/***/ "./lib/utils/to-function.js":
/*!**********************************!*\
  !*** ./lib/utils/to-function.js ***!
  \**********************************/
/***/ ((module) => {

// Const diag = require('../linalgebra/diag.js');

/**
* @callback MatrixCallback
* @returns <Array.<Array.<Number>>
*/

/**
* Tranforms:
** a 2d array into a function (() => array)
** a 1d array into a function (() => diag(array))
*@param {Array.<Number> | Array.<Array.<Number>>} array
*@returns {MatrixCallback}
*/

module.exports = function (array) {
	if (typeof (array) === 'function') {
		return array;
	}

	if (Array.isArray(array)) {
		return array;
	}

	throw (new Error('Only arrays and functions are authorized'));
};


/***/ }),

/***/ "./lib/utils/uniq.js":
/*!***************************!*\
  !*** ./lib/utils/uniq.js ***!
  \***************************/
/***/ ((module) => {

module.exports = function (array) {
	return array.filter((value, index) =>
		array.indexOf(value) === index
	);
};


/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/instance/getArgument.js":
/*!*************************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/instance/getArgument.js ***!
  \*************************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the argument of a Complex Number which is restricted to the interval [ 0, 2π ).<br><br>
 * 
 * The argument of the Complex Number is the angle between positive real-axis
 * and the vector representing the Complex Number on Complex plane.<br><br>
 * 
 * If the given Complex Number is considered as 0, returns undefined.
 * @memberof Complex
 * @instance
 * @returns {number} The argument of the Complex Number
 */
function getArgument() {
  var x = this.re;
  var y = this.im;
  var epsilon = 1 / (Math.pow(10, 15) * 2);

  if (Math.abs(x) < epsilon && Math.abs(y) < epsilon) {
    return undefined;
  }

  if (x === 0) {
    if (y > 0) {
      return Math.PI * 0.5;
    }

    return Math.PI * 1.5;
  }

  if (y === 0) {
    if (x > 0) {
      return 0;
    }

    return Math.PI;
  }

  if (x > 0 && y > 0) {
    return Math.atan(y / x);
  }

  if (x < 0 && y > 0) {
    return Math.PI - Math.atan(y / (x * -1));
  }

  if (x < 0 && y < 0) {
    return Math.PI + Math.atan(y * -1 / (x * -1));
  }

  return Math.PI * 2 - Math.atan(y * -1 / x);
}

module.exports = getArgument;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/instance/getImaginary.js":
/*!**************************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/instance/getImaginary.js ***!
  \**************************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Gets the imaginary part of a Complex Number.
 * @memberof Complex
 * @instance
 * @returns {number} The imaginary part of the Complex Number
 */
function getImaginary() {
  return this.im;
}

module.exports = getImaginary;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/instance/getModulus.js":
/*!************************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/instance/getModulus.js ***!
  \************************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the modulus of a Complex Number.<br><br>
 * 
 * The modulus of the complex number is the length of the vector
 * representing the complex number on complex plane.
 * @memberof Complex
 * @instance
 * @returns {number} The modulus of the Complex Number
 */
function getModulus() {
  return Math.sqrt(Math.pow(this.re, 2) + Math.pow(this.im, 2));
}

module.exports = getModulus;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/instance/getReal.js":
/*!*********************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/instance/getReal.js ***!
  \*********************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Gets the real part of a Complex Number.
 * @memberof Complex
 * @instance
 * @returns {number} The real part of the Complex Number
 */
function getReal() {
  return this.re;
}

module.exports = getReal;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/instance/toString.js":
/*!**********************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/instance/toString.js ***!
  \**********************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Gets the stringified and formatted Complex Number.
 * @memberof Complex
 * @instance
 * @returns {string} The stringified and formatted Complex Number
 */
function toString() {
  var re = this.re,
      im = this.im;

  if (Number.isNaN(re) || Number.isNaN(im)) {
    return 'NaN';
  }

  if (re === 0 && im === 0) {
    return '0';
  }

  if (re === 0) {
    if (im === 1) {
      return 'i';
    }

    if (im === -1) {
      return '-i';
    }

    return "".concat(im, "i");
  }

  if (im === 0) {
    return "".concat(re);
  }

  if (im > 0) {
    if (im === 1) {
      return "".concat(re, " + i");
    }

    return "".concat(re, " + ").concat(im, "i");
  }

  if (im === -1) {
    return "".concat(re, " - i");
  }

  return "".concat(re, " - ").concat(Math.abs(im), "i");
}

module.exports = toString;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/acos.js":
/*!****************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/acos.js ***!
  \****************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the inverse cosine of a Complex Number.
 * @memberof Complex
 * @static
 * @param {Complex} num - Any Complex Number
 * @returns {Complex} The result of inverse cosine function
 */
function acos(num) {
  return this.subtract(new this(Math.PI / 2), this.asin(num));
}

module.exports = acos;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/acot.js":
/*!****************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/acot.js ***!
  \****************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the inverse cotangent of a Complex Number.
 * The domain of this function is C / { i , -i , 0 }.<br><br>
 * 
 * If the argument is out of its domain, it returns Complex.NaN.
 * @memberof Complex
 * @static
 * @param {Complex} num - Any Complex Number except i, -i and 0
 * @returns {Complex} The result of inverse cotangent function
 */
function acot(num) {
  return this.atan(this.inverse(num));
}

module.exports = acot;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/acsc.js":
/*!****************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/acsc.js ***!
  \****************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the inverse cosecant of a Complex Number.
 * The domain of this function is C / { 0 }.<br><br>
 * 
 * If the argument is out of its domain, it returns Complex.NaN.
 * @memberof Complex
 * @static
 * @param {Complex} num - Any Complex Number except 0
 * @returns {Complex} The result of inverse cosecant function
 */
function acsc(num) {
  return this.asin(this.inverse(num));
}

module.exports = acsc;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/add.js":
/*!***************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/add.js ***!
  \***************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the sum of two Complex Number.
 * @memberof Complex
 * @static
 * @param {Complex} num1 - The Complex Number on the left of '+' operator.
 * @param {Complex} num2 - The Complex Number on the right of '+' operator.
 * @returns {Complex} The sum of two Complex Numbers
 */
function add(num1, num2) {
  if (!(num1 instanceof this) || !(num2 instanceof this)) {
    return this.NaN;
  }

  return new this(num1.re + num2.re, num1.im + num2.im);
}

module.exports = add;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/asec.js":
/*!****************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/asec.js ***!
  \****************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the inverse secant of a Complex Number.
 * The domain of this function is C / { 0 }.<br><br>
 * 
 * If the argument is out of its domain, it returns Complex.NaN.
 * @memberof Complex
 * @static
 * @param {Complex} num - Any Complex Number except 0
 * @returns {Complex} The result of inverse secant function
 */
function asec(num) {
  return this.acos(this.inverse(num));
}

module.exports = asec;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/asin.js":
/*!****************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/asin.js ***!
  \****************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the inverse sine of a Complex Number.
 * @memberof Complex
 * @static
 * @param {Complex} num - Any Complex Number
 * @returns {Complex} The result of inverse sine function
 */
function asin(num) {
  return this.multiply(new this(0, -1), this.log(this.add(this.multiply(new this(0, 1), num), this.pow(this.subtract(this.ONE, this.pow(num, 2)), 0.5))));
}

module.exports = asin;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/atan.js":
/*!****************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/atan.js ***!
  \****************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the inverse tangent of a Complex Number.
 * The domain of this function is C / { i , -i }.<br><br>
 * 
 * If the argument is out of its domain, it returns Complex.NaN.
 * @memberof Complex
 * @static
 * @param {Complex} num - Any Complex Number except i and -i
 * @returns {Complex} The result of inverse tangent function
 */
function atan(num) {
  return this.multiply(new this(0, 1 / 2), this.subtract(this.log(this.subtract(this.ONE, this.multiply(new this(0, 1), num))), this.log(this.add(this.ONE, this.multiply(new this(0, 1), num)))));
}

module.exports = atan;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/conjugate.js":
/*!*********************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/conjugate.js ***!
  \*********************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the complex conjugate of the Complex Number.
 * @memberof Complex
 * @static
 * @param {Complex} num - Complex number
 * @returns {Complex} The complex conjugate of the Complex Number
 */
function conjugate(num) {
  if (!(num instanceof this)) {
    return this.NaN;
  }

  return new this(num.getReal(), num.getImaginary() * -1);
}

module.exports = conjugate;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/cos.js":
/*!***************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/cos.js ***!
  \***************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the cosine of a Complex Number.
 * @memberof Complex
 * @static
 * @param {Complex} num - Any Complex Number
 * @returns {Complex} The result of cosine function
 */
function cos(num) {
  if (!(num instanceof this)) {
    return this.NaN;
  }

  var a = num.getReal();
  var b = num.getImaginary();
  return new this(Math.cos(a) * Math.cosh(b), Math.sin(a) * Math.sinh(b) * -1);
}

module.exports = cos;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/cot.js":
/*!***************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/cot.js ***!
  \***************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the cotangent of a Complex Number.
 * The domain of this function is C / { kπ/2 : k is any integer }.<br><br>
 * 
 * If the argument is out of its domain, it returns Complex.NaN.
 * @memberof Complex
 * @static
 * @param {Complex} num - Any Complex Number which is not the multiple of π/2
 * @returns {Complex} The result of cotangent function
 */
function cot(num) {
  return this.divide(this.ONE, this.tan(num));
}

module.exports = cot;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/csc.js":
/*!***************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/csc.js ***!
  \***************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the cosecant of a Complex Number.
 * The domain of this function is C / { kπ : k is any integer }.<br><br>
 * 
 * If the argument is out of its domain, it returns Complex.NaN.
 * @memberof Complex
 * @static
 * @param {Complex} num - Any Complex Number which is not the multiple of π
 * @returns {Complex} The result of cosecant function
 */
function csc(num) {
  return this.divide(this.ONE, this.sin(num));
}

module.exports = csc;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/divide.js":
/*!******************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/divide.js ***!
  \******************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the quotient of two Complex Number.<br><br>
 * 
 * Note that if the denominator is considered as 0,
 * returns Complex.NaN instead of Infinity.
 * @memberof Complex
 * @static
 * @param {Complex} num1 - The Complex Number on the left of '/' operator.
 * @param {Complex} num2 - The Complex Number on the right of '/' operator.
 * @returns {Complex} The quotient of two Complex Numbers
 */
function divide(num1, num2) {
  if (!(num1 instanceof this) || !(num2 instanceof this)) {
    return this.NaN;
  }

  var a = num1.re;
  var b = num1.im;
  var c = num2.re;
  var d = num2.im;

  if (Math.abs(c) < this.EPSILON && Math.abs(d) < this.EPSILON) {
    return this.NaN;
  }

  var denominator = Math.pow(c, 2) + Math.pow(d, 2);
  return new this((a * c + b * d) / denominator, (b * c - a * d) / denominator);
}

module.exports = divide;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/exp.js":
/*!***************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/exp.js ***!
  \***************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the exponential function with base E.
 * @memberof Complex
 * @static
 * @param {Complex} num - Exponent
 * @returns {Complex} The value of E to the power of num
 */
function exp(num) {
  if (!(num instanceof this)) {
    return this.NaN;
  }

  var re = num.getReal();
  var theta = num.getImaginary();
  var r = Math.exp(re);
  return new this(r * Math.cos(theta), r * Math.sin(theta));
}

module.exports = exp;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/inverse.js":
/*!*******************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/inverse.js ***!
  \*******************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the inverse of the Complex Number.
 * @memberof Complex
 * @static
 * @param {Complex} num - Complex Number
 * @returns {number} Inverse of the Complex Number
 */
function inverse(num) {
  if (!(num instanceof this)) {
    return this.NaN;
  }

  return this.divide(this.ONE, num);
}

module.exports = inverse;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/isEqual.js":
/*!*******************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/isEqual.js ***!
  \*******************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Determines whether two Complex Numbers are considered as identical.<br><br>
 * 
 * Two Complex Numbers are considered as identical if either
 * both are NaN or both real and imaginary parts are extremely closed.<br><br>
 * 
 * The test criterion is Math.abs(x - y) < 1 / (10 ** digit * 2).
 * For default value 15, it should be 5e-16.
 * That means if the difference of two numbers is less than 5e-16,
 * they are considered as same value.
 * @memberof Complex
 * @static
 * @param {Complex} num1 - Complex Number
 * @param {Complex} num2 - Complex Number
 * @param {number} [digit=15] - Number of significant digits
 * @returns {boolean} Returns true if two Complex Numbers are considered as identical
 */
function isEqual(num1, num2) {
  var digit = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : 15;

  if (!(num1 instanceof this) || !(num2 instanceof this)) {
    return false;
  }

  if (!Number.isInteger(digit) || digit < 0) {
    throw new Error('Invalid argument: Expected a non-negative integer digit');
  }

  var EPSILON = 1 / (Math.pow(10, digit) * 2);
  var a = num1.getReal();
  var b = num1.getImaginary();
  var c = num2.getReal();
  var d = num2.getImaginary();

  if (Number.isNaN(a) && Number.isNaN(b) && Number.isNaN(c) && Number.isNaN(d)) {
    return true;
  }

  return Math.abs(a - c) < EPSILON && Math.abs(b - d) < EPSILON;
}

module.exports = isEqual;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/isNaN.js":
/*!*****************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/isNaN.js ***!
  \*****************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Determines whether the Complex Number is NaN or not.
 * @memberof Complex
 * @static
 * @param {Complex} num - Any Complex number
 * @returns {boolean} Returns true if one of real and imaginary part are NaN
 */
function isNaN(num) {
  if (!(num instanceof this)) {
    return false;
  }

  var re = num.getReal();
  var im = num.getImaginary();

  if (Number.isNaN(re) || Number.isNaN(im)) {
    return true;
  }

  return false;
}

module.exports = isNaN;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/log.js":
/*!***************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/log.js ***!
  \***************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the natural log of the Complex Number.<br><br>
 * 
 * Note that complex log is a multivalued function,
 * and this function only provides the principal value by
 * restricting the imaginary part to the interval [0, 2π).
 * @memberof Complex
 * @static
 * @param {Complex} num - Complex Number
 * @returns {number} Natural log of the Complex Number
 */
function log(num) {
  if (!(num instanceof this)) {
    return this.NaN;
  }

  var r = num.getModulus();
  var theta = num.getArgument();

  if (r < this.EPSILON || theta === undefined) {
    return this.NaN;
  }

  return new this(Math.log(r), theta);
}

module.exports = log;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/multiply.js":
/*!********************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/multiply.js ***!
  \********************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the product of two Complex Number.
 * @memberof Complex
 * @static
 * @param {Complex} num1 - The Complex Number on the left of '*' operator.
 * @param {Complex} num2 - The Complex Number on the right of '*' operator.
 * @returns {Complex} The product of two Complex Numbers
 */
function multiply(num1, num2) {
  if (!(num1 instanceof this) || !(num2 instanceof this)) {
    return this.NaN;
  }

  var a = num1.re;
  var b = num1.im;
  var c = num2.re;
  var d = num2.im;
  return new this(a * c - b * d, a * d + b * c);
}

module.exports = multiply;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/pow.js":
/*!***************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/pow.js ***!
  \***************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the power of the Complex Number.
 * The exponent can be any real number or Complex Number<br><br>
 * 
 * You can find the k-th root of complex number by setting the exponent to 1 / k.
 * But you should know that it only returns one out of k possible solutions.
 * @memberof Complex
 * @static
 * @param {Complex} num - Base
 * @param {Complex|number} n - Exponent
 * @returns {Complex} The result of the exponentiation
 */
function pow(num, n) {
  if (!(num instanceof this) || typeof n !== 'number' && !(n instanceof this)) {
    return this.NaN;
  }

  if (typeof n === 'number') {
    if (!Number.isFinite(n) || Number.isNaN(n)) {
      return this.NaN;
    }

    if (n === 0) {
      return this.ONE;
    }

    if (this.isEqual(num, this.ZERO)) {
      return this.ZERO;
    }

    return this.exp(this.multiply(new this(n, 0), this.log(num)));
  }

  if (n instanceof this) {
    return this.exp(this.multiply(n, this.log(num)));
  }

  return this.NaN;
}

module.exports = pow;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/sec.js":
/*!***************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/sec.js ***!
  \***************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the secant of a Complex Number.
 * The domain of this function is C / { (k + 0.5)π : k is any integer }.<br><br>
 * 
 * If the argument is out of its domain, it returns Complex.NaN.
 * @memberof Complex
 * @static
 * @param {Complex} num - Any Complex Number which is not in the form of (k + 0.5)π
 * @returns {Complex} The result of secant function
 */
function sec(num) {
  return this.divide(this.ONE, this.cos(num));
}

module.exports = sec;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/sin.js":
/*!***************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/sin.js ***!
  \***************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the sine of a Complex Number.
 * @memberof Complex
 * @static
 * @param {Complex} num - Any Complex Number
 * @returns {Complex} The result of sine function
 */
function sin(num) {
  if (!(num instanceof this)) {
    return this.NaN;
  }

  var a = num.getReal();
  var b = num.getImaginary();
  return new this(Math.sin(a) * Math.cosh(b), Math.cos(a) * Math.sinh(b));
}

module.exports = sin;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/subtract.js":
/*!********************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/subtract.js ***!
  \********************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the difference of two Complex Number.
 * @memberof Complex
 * @static
 * @param {Complex} num1 - The Complex Number on the left of '-' operator.
 * @param {Complex} num2 - The Complex Number on the right of '-' operator.
 * @returns {Complex} The difference of two Complex Numbers
 */
function subtract(num1, num2) {
  if (!(num1 instanceof this) || !(num2 instanceof this)) {
    return this.NaN;
  }

  return new this(num1.re - num2.re, num1.im - num2.im);
}

module.exports = subtract;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/core/static/tan.js":
/*!***************************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/core/static/tan.js ***!
  \***************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the tangent of a Complex Number.
 * The domain of this function is C / { (k + 0.5)π : k is any integer }.<br><br>
 * 
 * If the argument is out of its domain, it returns Complex.NaN.
 * @memberof Complex
 * @static
 * @param {Complex} num - Any Complex Number which is not in the form of (k + 0.5)π
 * @returns {Complex} The result of tangent function
 */
function tan(num) {
  return this.divide(this.sin(num), this.cos(num));
}

module.exports = tan;

/***/ }),

/***/ "./node_modules/@rayyamhk/complex/lib/index.js":
/*!*****************************************************!*\
  !*** ./node_modules/@rayyamhk/complex/lib/index.js ***!
  \*****************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _typeof(obj) { "@babel/helpers - typeof"; if (typeof Symbol === "function" && typeof Symbol.iterator === "symbol") { _typeof = function _typeof(obj) { return typeof obj; }; } else { _typeof = function _typeof(obj) { return obj && typeof Symbol === "function" && obj.constructor === Symbol && obj !== Symbol.prototype ? "symbol" : typeof obj; }; } return _typeof(obj); }

/**
 * Creates a new Complex Number.
 * @namespace Complex
 * @class
 * @param {number} arg1 - The real part of the Complex Number
 * @param {number} arg2 - The imaginary part of the Complex Number
 */
function Complex(arg1, arg2) {
  var type1 = _typeof(arg1);

  var type2 = _typeof(arg2);

  if (type1 === 'number' && type2 === 'undefined') {
    if (Number.isNaN(arg1) || !Number.isFinite(arg1)) {
      this.re = NaN;
      this.im = NaN;
      return this;
    }

    this.re = arg1;
    this.im = 0;
    return this;
  }

  if (type1 === 'number' && type2 === 'number') {
    if (Number.isNaN(arg1) || Number.isNaN(arg2) || !Number.isFinite(arg1) || !Number.isFinite(arg2)) {
      this.re = NaN;
      this.im = NaN;
      return this;
    }

    this.re = arg1;
    this.im = arg2;
    return this;
  }

  this.re = NaN;
  this.im = NaN;
  return this;
}

module.exports = Complex;
Complex.prototype.getReal = __webpack_require__(/*! ./core/instance/getReal */ "./node_modules/@rayyamhk/complex/lib/core/instance/getReal.js");
Complex.prototype.getImaginary = __webpack_require__(/*! ./core/instance/getImaginary */ "./node_modules/@rayyamhk/complex/lib/core/instance/getImaginary.js");
Complex.prototype.getModulus = __webpack_require__(/*! ./core/instance/getModulus */ "./node_modules/@rayyamhk/complex/lib/core/instance/getModulus.js");
Complex.prototype.getArgument = __webpack_require__(/*! ./core/instance/getArgument */ "./node_modules/@rayyamhk/complex/lib/core/instance/getArgument.js");
Complex.prototype.toString = __webpack_require__(/*! ./core/instance/toString */ "./node_modules/@rayyamhk/complex/lib/core/instance/toString.js");
Complex.isNaN = __webpack_require__(/*! ./core/static/isNaN */ "./node_modules/@rayyamhk/complex/lib/core/static/isNaN.js");
Complex.isEqual = __webpack_require__(/*! ./core/static/isEqual */ "./node_modules/@rayyamhk/complex/lib/core/static/isEqual.js");
Complex.conjugate = __webpack_require__(/*! ./core/static/conjugate */ "./node_modules/@rayyamhk/complex/lib/core/static/conjugate.js");
Complex.inverse = __webpack_require__(/*! ./core/static/inverse */ "./node_modules/@rayyamhk/complex/lib/core/static/inverse.js");
Complex.add = __webpack_require__(/*! ./core/static/add */ "./node_modules/@rayyamhk/complex/lib/core/static/add.js");
Complex.subtract = __webpack_require__(/*! ./core/static/subtract */ "./node_modules/@rayyamhk/complex/lib/core/static/subtract.js");
Complex.multiply = __webpack_require__(/*! ./core/static/multiply */ "./node_modules/@rayyamhk/complex/lib/core/static/multiply.js");
Complex.divide = __webpack_require__(/*! ./core/static/divide */ "./node_modules/@rayyamhk/complex/lib/core/static/divide.js");
Complex.exp = __webpack_require__(/*! ./core/static/exp */ "./node_modules/@rayyamhk/complex/lib/core/static/exp.js");
Complex.log = __webpack_require__(/*! ./core/static/log */ "./node_modules/@rayyamhk/complex/lib/core/static/log.js");
Complex.pow = __webpack_require__(/*! ./core/static/pow */ "./node_modules/@rayyamhk/complex/lib/core/static/pow.js");
Complex.sin = __webpack_require__(/*! ./core/static/sin */ "./node_modules/@rayyamhk/complex/lib/core/static/sin.js");
Complex.cos = __webpack_require__(/*! ./core/static/cos */ "./node_modules/@rayyamhk/complex/lib/core/static/cos.js");
Complex.tan = __webpack_require__(/*! ./core/static/tan */ "./node_modules/@rayyamhk/complex/lib/core/static/tan.js");
Complex.csc = __webpack_require__(/*! ./core/static/csc */ "./node_modules/@rayyamhk/complex/lib/core/static/csc.js");
Complex.sec = __webpack_require__(/*! ./core/static/sec */ "./node_modules/@rayyamhk/complex/lib/core/static/sec.js");
Complex.cot = __webpack_require__(/*! ./core/static/cot */ "./node_modules/@rayyamhk/complex/lib/core/static/cot.js");
Complex.asin = __webpack_require__(/*! ./core/static/asin */ "./node_modules/@rayyamhk/complex/lib/core/static/asin.js");
Complex.acos = __webpack_require__(/*! ./core/static/acos */ "./node_modules/@rayyamhk/complex/lib/core/static/acos.js");
Complex.atan = __webpack_require__(/*! ./core/static/atan */ "./node_modules/@rayyamhk/complex/lib/core/static/atan.js");
Complex.acsc = __webpack_require__(/*! ./core/static/acsc */ "./node_modules/@rayyamhk/complex/lib/core/static/acsc.js");
Complex.asec = __webpack_require__(/*! ./core/static/asec */ "./node_modules/@rayyamhk/complex/lib/core/static/asec.js");
Complex.acot = __webpack_require__(/*! ./core/static/acot */ "./node_modules/@rayyamhk/complex/lib/core/static/acot.js");
/**
 * It represents NaN in this library. It is equivalent to new Complex(NaN).<br><br>
 * 
 * It is important to know that this library does not introduce the concept of Complex Infinity,
 * all Infinity in this library are represented by Complex.NaN.
 * @static
 */

Complex.NaN = new Complex(NaN);
/** @static */

Complex.ONE = new Complex(1);
/** @static */

Complex.ZERO = new Complex(0);
/** @static */

Complex.PI = new Complex(Math.PI);
/** @static */

Complex.E = new Complex(Math.E);
/**
 * It represents the value of 5e-16, which is the smallest number considered as non-zero in this library.
 * In the other words, any number less than Complex.EPSILON is considered as 0.<br><br>
 * 
 * Note that Complex.EPSILON is number instead of instance of Complex.
 * @static
 */

Complex.EPSILON = 1 / (Math.pow(10, 15) * 2);

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/Error.js":
/*!****************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/Error.js ***!
  \****************************************************/
/***/ ((module) => {

"use strict";


module.exports = {
  INVALID_ARRAY: 'Invalid argument: Received a non-array argument',
  INVALID_MATRIX: 'Invalid argument: Received an invalid matrix',
  INVALID_SQUARE_MATRIX: 'Invalid argument: Received a non-square matrix',
  INVALID_UPPER_TRIANGULAR_MATRIX: 'Invalid argument: Received a non upper-triangular matrix',
  INVALID_LOWER_TRIANGULAR_MATRIX: 'Invalid argument: Received a non lower-triangular matrix',
  INVALID_EXPONENT: 'Invalid argument: Expected a non-negative integer exponent',
  INVALID_ROW_COL: 'Invalid argument: Expected non-negative integer row and column',
  INVALID_ROW: 'Invalid argument: Expected non-negative integer row',
  INVALID_COLUMN: 'Invalid argument: Expected non-negative integer column',
  INVALID_ROWS_EXPRESSION: 'Invalid argument: Received invalid rows expression',
  INVALID_COLUMNS_EXPRESSION: 'Invalid argument: Received invalid columns expression',
  INVALID_P_NORM: 'Invalid argument: Received invalid p-norm',
  OVERFLOW_INDEX: 'Invalid argument: Matrix index overflow',
  OVERFLOW_COLUMN: 'Invalid argument: Column index overflow',
  OVERFLOW_ROW: 'Invalid argument: Row index overflow',
  NO_UNIQUE_SOLUTION: 'Arithmetic Exception: The system has no unique solution',
  SIZE_INCOMPATIBLE: 'Invalid argument: Matrix size-incompatible',
  SINGULAR_MATRIX: 'Arithmetic Exception: The matrix is not invertible',
  EXPECTED_STRING_NUMBER_AT_POS_1_2: 'Invalid argument: Expected a string or a number at arguments[1] and arguments[2]',
  EXPECTED_ARRAY_OF_NUMBERS_OR_MATRICES: 'Invalid argument: Expected either an array of numbers or an array of square matrices'
};

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/decompositions/LU.js":
/*!*********************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/decompositions/LU.js ***!
  \*********************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX;
/**
 * Calculates the LUP decomposition of the Matrix,
 * where L is lower triangular matrix which diagonal entries are always 1,
 * U is upper triangular matrix, and P is permutation matrix.<br><br>
 * 
 * It is implemented using Gaussian Elimination with Partial Pivoting in order to
 * reduce the error caused by floating-point arithmetic.<br><br>
 * 
 * Note that if optimized is true, P is a Permutation Array and both L and U are merged
 * into one matrix in order to improve performance.
 * @memberof Matrix
 * @static
 * @param {Matrix} A - Any matrix
 * @param {boolean} [optimized=false] - Returns [P, LU] if it is true, [P, L, U] if it is false
 * @returns {Matrix[]} The LUP decomposition of Matrix
 */


function LU(A) {
  var optimized = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : false;

  if (!(A instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  var _A$size = A.size(),
      _A$size2 = _slicedToArray(_A$size, 2),
      row = _A$size2[0],
      col = _A$size2[1];

  var size = Math.min(row, col);
  var EPSILON = 1 / (Math.pow(10, A._digit) * 2);
  var permutation = initPermutation(row);

  var copy = this.clone(A)._matrix;

  for (var i = 0; i < row - 1; i++) {
    var currentCol = Math.min(i, col); // apply Partial Pivoting

    PartialPivoting(copy, permutation, currentCol, row, col);
    var ith = permutation[i];
    var pivot = copy[ith][currentCol];

    if (Math.abs(pivot) < EPSILON) {
      continue;
    }

    for (var j = i + 1; j < row; j++) {
      var jth = permutation[j];
      var entry = copy[jth][currentCol];

      if (Math.abs(entry) >= EPSILON) {
        var factor = entry / pivot;

        for (var k = currentCol; k < col; k++) {
          copy[jth][k] -= factor * copy[ith][k];
        }

        copy[jth][currentCol] = factor;
      }
    }
  }

  var result = new Array(row);

  for (var _i2 = 0; _i2 < row; _i2++) {
    result[_i2] = copy[permutation[_i2]];
  }

  if (optimized) {
    return [permutation, new this(result)];
  }

  var P = this.generate(row, row, function (i, j) {
    var idx = permutation[i];

    if (j === idx) {
      return 1;
    }

    return 0;
  });
  var L = this.generate(row, size, function (i, j) {
    if (i === j) {
      return 1;
    }

    if (i < j) {
      return 0;
    }

    return result[i][j];
  });
  var U = this.generate(size, col, function (i, j) {
    if (i > j) {
      return 0;
    }

    return result[i][j];
  });
  return [P, L, U];
}

;

function initPermutation(size) {
  var permutation = new Array(size);

  for (var i = 0; i < size; i++) {
    permutation[i] = i;
  }

  return permutation;
}

function PartialPivoting(matrix, permutation, pos, row, col) {
  var currentCol = Math.min(pos, col);
  var maxIdx = pos;
  var max = Math.abs(matrix[permutation[pos]][currentCol]);

  for (var i = pos + 1; i < row; i++) {
    var value = Math.abs(matrix[permutation[i]][currentCol]);

    if (value > max) {
      maxIdx = i;
      max = value;
    }
  }

  var t = permutation[pos];
  permutation[pos] = permutation[maxIdx];
  permutation[maxIdx] = t;
}

module.exports = LU;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/decompositions/QR.js":
/*!*********************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/decompositions/QR.js ***!
  \*********************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX;
/**
 * Calculates the QR decomposition of the Matrix
 * where Q is orthogonal matrix, R is upper triangular matrix.<br><br>
 * 
 * The algorithm is implemented using Householder Transform instead of Gram–Schmidt process
 * because the Householder Transform is more numerically stable.
 * @memberof Matrix
 * @static
 * @param {Matrix} A - Any matrix
 * @returns {Matrix[]} The QR decomposition of matrix in the form of [Q, R]
 */


function QR(A) {
  if (!(A instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  var _A$size = A.size(),
      _A$size2 = _slicedToArray(_A$size, 2),
      row = _A$size2[0],
      col = _A$size2[1];

  var size = Math.min(row, col);
  var EPSILON = 1 / (Math.pow(10, A._digit) * 2);

  var matrixR = this.clone(A)._matrix;

  var matrixQ = this.identity(row)._matrix;

  for (var j = 0; j < size; j++) {
    // if all entries below main diagonal are considered as zero, skip this round
    var skip = true;

    for (var i = j + 1; i < row; i++) {
      if (Math.abs(matrixR[i][j]) >= EPSILON) {
        skip = false;
        break;
      }
    }

    if (!skip) {
      // Apply Householder transform
      var norm = 0;

      for (var _i2 = j; _i2 < row; _i2++) {
        norm += Math.pow(matrixR[_i2][j], 2);
      }

      norm = Math.sqrt(norm); // reduce floating point arithmatic error

      var s = -1;

      if (matrixR[j][j] < 0) {
        s = 1;
      }

      var u1 = matrixR[j][j] - s * norm;
      var w = new Array(row - j);

      for (var _i3 = 0; _i3 < row - j; _i3++) {
        w[_i3] = matrixR[_i3 + j][j] / u1;
      }

      w[0] = 1;
      var tau = -1 * s * u1 / norm;
      var subR = new Array(row - j);

      for (var _i4 = 0; _i4 < row - j; _i4++) {
        var newRow = new Array(col);

        for (var k = 0; k < col; k++) {
          newRow[k] = matrixR[j + _i4][k];
        }

        subR[_i4] = newRow;
      }

      for (var _i5 = j; _i5 < row; _i5++) {
        for (var _k = 0; _k < col; _k++) {
          var summation = 0;

          for (var m = 0; m < row - j; m++) {
            summation += subR[m][_k] * w[m];
          }

          matrixR[_i5][_k] = subR[_i5 - j][_k] - tau * w[_i5 - j] * summation;
        }
      }

      var subQ = new Array(row);

      for (var _i6 = 0; _i6 < row; _i6++) {
        var _newRow = new Array(row - j);

        for (var _k2 = 0; _k2 < row - j; _k2++) {
          _newRow[_k2] = matrixQ[_i6][j + _k2];
        }

        subQ[_i6] = _newRow;
      }

      for (var _i7 = 0; _i7 < row; _i7++) {
        for (var _k3 = j; _k3 < row; _k3++) {
          var _summation = 0;

          for (var _m = 0; _m < row - j; _m++) {
            _summation += subQ[_i7][_m] * w[_m];
          }

          matrixQ[_i7][_k3] = subQ[_i7][_k3 - j] - tau * w[_k3 - j] * _summation;
        }
      }
    }
  }

  for (var _i8 = 0; _i8 < row; _i8++) {
    for (var _j = 0; _j < col; _j++) {
      if (_i8 > _j) {
        matrixR[_i8][_j] = 0;
      }
    }
  }

  return [new this(matrixQ), new this(matrixR)];
}

;
module.exports = QR;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/linear-equations/backward.js":
/*!*****************************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/linear-equations/backward.js ***!
  \*****************************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var empty = __webpack_require__(/*! ../../util/empty */ "./node_modules/@rayyamhk/matrix/lib/util/empty.js");

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX,
    INVALID_UPPER_TRIANGULAR_MATRIX = _require.INVALID_UPPER_TRIANGULAR_MATRIX,
    INVALID_SQUARE_MATRIX = _require.INVALID_SQUARE_MATRIX,
    SIZE_INCOMPATIBLE = _require.SIZE_INCOMPATIBLE,
    NO_UNIQUE_SOLUTION = _require.NO_UNIQUE_SOLUTION;
/**
* Solve system of linear equations Ux = y using backward substitution,
* where U is an upper triangular matrix.
* If there is no unique solutions, an error is thrown.
* @memberof Matrix
* @static
* @param {Matrix} U - Any n x n upper triangular Matrix
* @param {Matrix} y - Any n x 1 Matrix
* @returns {Matrix} n x 1 Matrix which is the solution of Ux = y
*/


function backward(U, y) {
  if (!(U instanceof this) || !(y instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  if (!U.isUpperTriangular()) {
    throw new Error(INVALID_UPPER_TRIANGULAR_MATRIX);
  }

  if (!U.isSquare()) {
    throw new Error(INVALID_SQUARE_MATRIX);
  }

  var size = U.size()[0];

  var _y$size = y.size(),
      _y$size2 = _slicedToArray(_y$size, 2),
      yrow = _y$size2[0],
      ycol = _y$size2[1];

  var matrixU = U._matrix;
  var matrixY = y._matrix;

  if (yrow !== size || ycol !== 1) {
    throw new Error(SIZE_INCOMPATIBLE);
  }

  var EPSILON = 1 / (Math.pow(10, U._digit) * 2);

  for (var i = 0; i < size; i++) {
    if (Math.abs(matrixU[i][i]) < EPSILON) {
      throw new Error(NO_UNIQUE_SOLUTION);
    }
  }

  var coefficients = empty(size, 1);

  for (var _i2 = size - 1; _i2 >= 0; _i2--) {
    var summation = 0;

    for (var j = _i2 + 1; j < size; j++) {
      summation += coefficients[j][0] * matrixU[_i2][j];
    }

    coefficients[_i2][0] = (matrixY[_i2][0] - summation) / matrixU[_i2][_i2];
  }

  return new this(coefficients);
}

;
module.exports = backward;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/linear-equations/forward.js":
/*!****************************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/linear-equations/forward.js ***!
  \****************************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var empty = __webpack_require__(/*! ../../util/empty */ "./node_modules/@rayyamhk/matrix/lib/util/empty.js");

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX,
    INVALID_LOWER_TRIANGULAR_MATRIX = _require.INVALID_LOWER_TRIANGULAR_MATRIX,
    INVALID_SQUARE_MATRIX = _require.INVALID_SQUARE_MATRIX,
    SIZE_INCOMPATIBLE = _require.SIZE_INCOMPATIBLE,
    NO_UNIQUE_SOLUTION = _require.NO_UNIQUE_SOLUTION;
/**
 * Solve system of linear equations Lx = y using forward substitution,
 * where L is a lower triangular matrix.
 * If there is no unique solutions, an error is thrown.
 * @memberof Matrix
 * @static
 * @param {Matrix} L - Any n x n lower triangular Matrix
 * @param {Matrix} y - Any n x 1 Matrix
 * @returns {Matrix} n x 1 Matrix which is the solution of Lx = y
 */


function forward(L, y) {
  if (!(L instanceof this) || !(y instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  if (!L.isLowerTriangular()) {
    throw new Error(INVALID_LOWER_TRIANGULAR_MATRIX);
  }

  if (!L.isSquare()) {
    throw new Error(INVALID_SQUARE_MATRIX);
  }

  var size = L.size()[0];

  var _y$size = y.size(),
      _y$size2 = _slicedToArray(_y$size, 2),
      yrow = _y$size2[0],
      ycol = _y$size2[1];

  var matrixL = L._matrix;
  var matrixY = y._matrix;

  if (size !== yrow || ycol !== 1) {
    throw new Error(SIZE_INCOMPATIBLE);
  }

  var EPSILON = 1 / (Math.pow(10, L._digit) * 2);

  for (var i = 0; i < size; i++) {
    if (Math.abs(matrixL[i][i]) < EPSILON) {
      throw new Error(NO_UNIQUE_SOLUTION);
    }
  }

  var coefficients = empty(size, 1);

  for (var _i2 = 0; _i2 < size; _i2++) {
    var summation = 0;

    for (var j = 0; j < _i2; j++) {
      summation += coefficients[j][0] * matrixL[_i2][j];
    }

    coefficients[_i2][0] = (matrixY[_i2][0] - summation) / matrixL[_i2][_i2];
  }

  return new this(coefficients);
}

;
module.exports = forward;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/linear-equations/solve.js":
/*!**************************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/linear-equations/solve.js ***!
  \**************************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX,
    NO_UNIQUE_SOLUTION = _require.NO_UNIQUE_SOLUTION,
    INVALID_SQUARE_MATRIX = _require.INVALID_SQUARE_MATRIX,
    SIZE_INCOMPATIBLE = _require.SIZE_INCOMPATIBLE;
/**
 * Solve system of linear equations Ax = y using LU decomposition.
 * If there is no unique solutions, an error is thrown.
 * @memberof Matrix
 * @static
 * @param {Matrix} L - Any n x n square Matrix
 * @param {Matrix} y - Any n x 1 Matrix
 * @returns {Matrix} n x 1 Matrix which is the solution of Ax = y
 */


function solve(A, b) {
  if (!(A instanceof this) || !(b instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  if (!A.isSquare()) {
    throw new Error(INVALID_SQUARE_MATRIX);
  }

  var _A$size = A.size(),
      _A$size2 = _slicedToArray(_A$size, 2),
      aRow = _A$size2[0],
      aCol = _A$size2[1];

  var _b$size = b.size(),
      _b$size2 = _slicedToArray(_b$size, 2),
      bRow = _b$size2[0],
      bCol = _b$size2[1];

  if (aCol !== bRow || bCol !== 1) {
    throw new Error(SIZE_INCOMPATIBLE);
  }

  var EPSILON = 1 / (Math.pow(10, A._digit) * 2);

  var _this$LU = this.LU(A, true),
      _this$LU2 = _slicedToArray(_this$LU, 2),
      P = _this$LU2[0],
      LU = _this$LU2[1];

  var matrixLU = LU._matrix;
  var matrixB = b._matrix;

  for (var i = aRow - 1; i >= 0; i--) {
    if (Math.abs(matrixLU[i][i]) < EPSILON) {
      throw new Error(NO_UNIQUE_SOLUTION);
    }
  }

  var clonedVector = new Array(bRow);
  var coefficients = new Array(bRow);

  for (var _i2 = 0; _i2 < bRow; _i2++) {
    // eslint-disable-next-line prefer-destructuring
    clonedVector[_i2] = matrixB[P[_i2]][0];
  }

  for (var _i3 = 0; _i3 < aRow; _i3++) {
    var summation = 0;

    for (var j = 0; j < _i3; j++) {
      summation += coefficients[j] * matrixLU[_i3][j];
    }

    coefficients[_i3] = clonedVector[_i3] - summation;
  }

  for (var _i4 = aRow - 1; _i4 >= 0; _i4--) {
    var _summation = 0;

    for (var _j = _i4 + 1; _j < aRow; _j++) {
      _summation += matrixLU[_i4][_j] * clonedVector[_j];
    }

    clonedVector[_i4] = (coefficients[_i4] - _summation) / matrixLU[_i4][_i4];
  }

  for (var _i5 = 0; _i5 < bRow; _i5++) {
    coefficients[_i5] = [clonedVector[_i5]];
  }

  return new this(coefficients);
}

;
module.exports = solve;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/operations/add.js":
/*!******************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/operations/add.js ***!
  \******************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX,
    SIZE_INCOMPATIBLE = _require.SIZE_INCOMPATIBLE;
/**
 * Calculates the sum of two Matrices.
 * @memberof Matrix
 * @static
 * @param {Matrix} A - Any Matrix
 * @param {Matrix} B - Any Matrix that has same size with A
 * @returns {Matrix} The sum of two Matrices
 */


function add(A, B) {
  if (!(A instanceof this) || !(B instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  var _A$size = A.size(),
      _A$size2 = _slicedToArray(_A$size, 2),
      row = _A$size2[0],
      col = _A$size2[1];

  var _B$size = B.size(),
      _B$size2 = _slicedToArray(_B$size, 2),
      row2 = _B$size2[0],
      col2 = _B$size2[1];

  if (row !== row2 || col !== col2) {
    throw new Error(SIZE_INCOMPATIBLE);
  }

  var matrix1 = A._matrix;
  var matrix2 = B._matrix;
  return this.generate(row, col, function (i, j) {
    return matrix1[i][j] + matrix2[i][j];
  });
}

;
module.exports = add;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/operations/inverse.js":
/*!**********************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/operations/inverse.js ***!
  \**********************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX,
    INVALID_SQUARE_MATRIX = _require.INVALID_SQUARE_MATRIX,
    SINGULAR_MATRIX = _require.SINGULAR_MATRIX;

var Matrix = __webpack_require__(/*! ../.. */ "./node_modules/@rayyamhk/matrix/lib/index.js");
/**
 * Find the inverse of non-singular matrix using Elementary Row Operations.
 * If the matrix is singular, an error is thrown.
 * @memberof Matrix
 * @static
 * @param {Matrix} A - Any square Matrix
 * @returns {Matrix} The inverse of A
 */


function inverse(A) {
  if (!(A instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  if (!A.isSquare()) {
    throw new Error(INVALID_SQUARE_MATRIX);
  }

  var size = A.size()[0];

  if (size === 0) {
    // inverse of 0x0 matrix is itself
    return new Matrix([]);
  }

  var EPSILON = 1 / (Math.pow(10, A._digit) * 2);

  var inv = this.identity(size)._matrix;

  var clone = this.clone(A)._matrix;

  var permutation = initPermutation(size); // iterate each column

  for (var j = 0; j < size; j++) {
    var pivotIdx = j;
    var pivot = clone[permutation[j]][j];

    while (Math.abs(pivot) < EPSILON && pivotIdx < size - 1) {
      pivotIdx++;
      pivot = clone[permutation[pivotIdx]][j];
    }

    if (Math.abs(pivot) < EPSILON) {
      throw new Error(SINGULAR_MATRIX);
    }

    if (j !== pivotIdx) {
      var temp = permutation[j];
      permutation[j] = permutation[pivotIdx];
      permutation[pivotIdx] = temp;
    }

    var pivotRow = permutation[j]; // the pivot is guaranteed to be non-zero

    for (var i = 0; i < size; i++) {
      var ith = permutation[i];

      if (i === j) {
        for (var k = 0; k < size; k++) {
          if (k === j) {
            clone[ith][k] = 1;
          }

          if (k > j) {
            clone[ith][k] /= pivot;
          }

          inv[ith][k] /= pivot;
        }

        pivot = 1;
      }

      if (i !== j && Math.abs(clone[ith][j]) >= EPSILON) {
        var factor = clone[ith][j] / pivot;

        for (var _k = 0; _k < size; _k++) {
          if (_k === j) {
            clone[ith][_k] = 0;
          }

          if (_k > j) {
            clone[ith][_k] -= factor * clone[pivotRow][_k];
          }

          inv[ith][_k] -= factor * inv[pivotRow][_k];
        }
      }
    }
  }

  for (var _i = 0; _i < size; _i++) {
    clone[_i] = inv[permutation[_i]];
  }

  return new this(clone);
}

;

function initPermutation(size) {
  var permutation = new Array(size);

  for (var i = 0; i < size; i++) {
    permutation[i] = i;
  }

  return permutation;
}

module.exports = inverse;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/operations/multiply.js":
/*!***********************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/operations/multiply.js ***!
  \***********************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var empty = __webpack_require__(/*! ../../util/empty */ "./node_modules/@rayyamhk/matrix/lib/util/empty.js");

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX,
    SIZE_INCOMPATIBLE = _require.SIZE_INCOMPATIBLE;
/**
 * Calculates the product of two Matrices.
 * @memberof Matrix
 * @static
 * @param {Matrix} A - Any Matrix
 * @param {Matrix} B - Any Matrix that is size-compatible with A
 * @returns {Matrix} The product of two Matrices
 */


function multiply(A, B) {
  if (!(A instanceof this) || !(B instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  var _A$size = A.size(),
      _A$size2 = _slicedToArray(_A$size, 2),
      Arow = _A$size2[0],
      Acol = _A$size2[1];

  var _B$size = B.size(),
      _B$size2 = _slicedToArray(_B$size, 2),
      Brow = _B$size2[0],
      Bcol = _B$size2[1];

  if (Acol !== Brow) {
    throw new Error(SIZE_INCOMPATIBLE);
  }

  var matrixA = A._matrix;
  var matrixB = B._matrix;
  var result = empty(Arow, Bcol);

  for (var i = 0; i < Arow; i++) {
    for (var j = 0; j < Bcol; j++) {
      result[i][j] = 0;

      for (var k = 0; k < Brow; k++) {
        result[i][j] += matrixA[i][k] * matrixB[k][j];
      }
    }
  }

  return new this(result);
}

;
module.exports = multiply;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/operations/pow.js":
/*!******************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/operations/pow.js ***!
  \******************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX,
    INVALID_SQUARE_MATRIX = _require.INVALID_SQUARE_MATRIX,
    INVALID_EXPONENT = _require.INVALID_EXPONENT;
/**
 * Calculates the power of any square matrix.
 * The algorithm is implemented recursively.
 * @memberof Matrix
 * @static
 * @param {Matrix} A - Any square Matrix
 * @param {number} exponent - Any Non-negative integer
 * @returns {Matrix} The power of A
 */


function pow(A, exponent) {
  if (!(A instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  if (!A.isSquare()) {
    throw new Error(INVALID_SQUARE_MATRIX);
  }

  if (!Number.isInteger(exponent) || exponent < 0) {
    throw new Error(INVALID_EXPONENT);
  }

  var size = A.size()[0];

  if (exponent === 0) {
    return this.identity(size);
  }

  if (exponent === 1) {
    return this.clone(A);
  }

  if (exponent % 2 === 0) {
    var _temp = this.pow(A, exponent / 2);

    return this.multiply(_temp, _temp);
  }

  var temp = this.pow(A, (exponent - 1) / 2);
  return this.multiply(this.multiply(temp, temp), A);
}

;
module.exports = pow;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/operations/subtract.js":
/*!***********************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/operations/subtract.js ***!
  \***********************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    SIZE_INCOMPATIBLE = _require.SIZE_INCOMPATIBLE,
    INVALID_MATRIX = _require.INVALID_MATRIX;
/**
 * Calculates the difference of two Matrices.
 * @memberof Matrix
 * @static
 * @param {Matrix} A - Any Matrix
 * @param {Matrix} B - Any Matrix that has same size with A
 * @returns {Matrix} The difference of two Matrices
 */


module.exports = function subtract(A, B) {
  if (!(A instanceof this) || !(B instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  var _A$size = A.size(),
      _A$size2 = _slicedToArray(_A$size, 2),
      row = _A$size2[0],
      col = _A$size2[1];

  var _B$size = B.size(),
      _B$size2 = _slicedToArray(_B$size, 2),
      row2 = _B$size2[0],
      col2 = _B$size2[1];

  if (row !== row2 || col !== col2) {
    throw new Error(SIZE_INCOMPATIBLE);
  }

  var matrix1 = A._matrix;
  var matrix2 = B._matrix;
  return this.generate(row, col, function (i, j) {
    return matrix1[i][j] - matrix2[i][j];
  });
};

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/operations/transpose.js":
/*!************************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/operations/transpose.js ***!
  \************************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX;
/**
 * Find the transpose of a matrix.
 * @memberof Matrix
 * @static
 * @param { Matrix } A - Any Matrix
 * @returns { Matrix } Returns transpose of A
 */


function transpose(A) {
  if (!(A instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  var _A$size = A.size(),
      _A$size2 = _slicedToArray(_A$size, 2),
      row = _A$size2[0],
      col = _A$size2[1];

  var matrix = A._matrix;
  return this.generate(col, row, function (i, j) {
    return matrix[j][i];
  });
}

;
module.exports = transpose;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/properties/cond.js":
/*!*******************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/properties/cond.js ***!
  \*******************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


var Matrix = __webpack_require__(/*! ../.. */ "./node_modules/@rayyamhk/matrix/lib/index.js");

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_P_NORM = _require.INVALID_P_NORM,
    SINGULAR_MATRIX = _require.SINGULAR_MATRIX,
    INVALID_SQUARE_MATRIX = _require.INVALID_SQUARE_MATRIX;
/**
 * Calculations the condition number of square Matrix
 * with respect to the choice of Matrix norm. 
 * If the Matrix is singular, returns Infinity.<br><br>
 * The condition number is not cached.
 * @memberof Matrix
 * @instance
 * @param {(1|2|Infinity|'F')} p - Type of Matrix norm
 * @returns {number} The condition number of Matrix
 */


function cond() {
  var p = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : 2;

  if (p !== 1 && p !== 2 && p !== Infinity && p !== 'F') {
    throw new Error(INVALID_P_NORM);
  }

  if (!this.isSquare()) {
    throw new Error(INVALID_SQUARE_MATRIX);
  }

  try {
    var inverse = Matrix.inverse(this);
    return inverse.norm(p) * this.norm(p);
  } catch (error) {
    if (error.message === SINGULAR_MATRIX) {
      return Infinity;
    }

    throw error;
  }
}

;
module.exports = cond;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/properties/det.js":
/*!******************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/properties/det.js ***!
  \******************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

/* eslint-disable prefer-destructuring */
var Matrix = __webpack_require__(/*! ../.. */ "./node_modules/@rayyamhk/matrix/lib/index.js");

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_SQUARE_MATRIX = _require.INVALID_SQUARE_MATRIX;
/**
 * Calculates the determinant of square Matrix.
 * If the Matrix size is larger than 3, it calculates the determinant using
 * LU decomposition, otherwise, using Leibniz Formula.<br><br>
 * The determinant is cached.
 * @memberof Matrix
 * @instance
 * @returns {number} Returns the determinant of square matrirx
 */


function det() {
  if (!this.isSquare()) {
    throw new Error(INVALID_SQUARE_MATRIX);
  }

  if (this._det !== undefined) {
    return this._det;
  }

  var matrix = this._matrix;
  var size = matrix.length;

  if (size === 0) {
    this._det = 1;
    return 1; // the determinant of 0x0 matrix must be 1
  }

  if (size === 1) {
    this._det = matrix[0][0];
    return this._det;
  }

  if (size === 2) {
    this._det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    return this._det;
  }

  if (size === 3) {
    this._det = matrix[0][0] * matrix[1][1] * matrix[2][2] + matrix[0][1] * matrix[1][2] * matrix[2][0] + matrix[0][2] * matrix[1][0] * matrix[2][1] - matrix[0][2] * matrix[1][1] * matrix[2][0] - matrix[0][1] * matrix[1][0] * matrix[2][2] - matrix[0][0] * matrix[1][2] * matrix[2][1];
    return this._det;
  }

  var _Matrix$LU = Matrix.LU(this, true),
      _Matrix$LU2 = _slicedToArray(_Matrix$LU, 2),
      P = _Matrix$LU2[0],
      LU = _Matrix$LU2[1];

  var matrixLU = LU._matrix; // count whether the number of permutations <swap> is odd or even
  // O(n^2)

  var swap = 0;

  for (var i = 0; i < size; i++) {
    if (P[i] === i) {
      continue;
    }

    while (P[i] !== i) {
      var target = P[i];
      P[i] = P[target];
      P[target] = target;
      swap++;
    }
  }

  var result = 1;

  for (var _i2 = 0; _i2 < size; _i2++) {
    result *= matrixLU[_i2][_i2];
  }

  if (swap % 2 === 1) {
    this._det = result * -1;
    return this._det;
  }

  this._det = result;
  return result;
}

;
module.exports = det;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/properties/eigenvalues.js":
/*!**************************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/properties/eigenvalues.js ***!
  \**************************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

/* eslint-disable no-param-reassign */
// reference: https://people.inf.ethz.ch/arbenz/ewp/Lnotes/chapter4.pdf
var Complex = __webpack_require__(/*! @rayyamhk/complex */ "./node_modules/@rayyamhk/complex/lib/index.js");

var Matrix = __webpack_require__(/*! ../.. */ "./node_modules/@rayyamhk/matrix/lib/index.js");

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_SQUARE_MATRIX = _require.INVALID_SQUARE_MATRIX;
/**
 * Calculates the eigenvalues of any square Matrix using QR Algorithm.<br><br>
 * 
 * The eigenvalues can be either real number or complex number.
 * Note that all eigenvalues are instance of Complex,
 * for more details please visit [Complex.js]{@link https://github.com/rayyamhk/Complex.js}.<br><br>
 * 
 * The eigenvalues are cached.
 * @memberof Matrix
 * @instance
 * @returns {Complex[]} Array of eigenvalues
 */


function eigenvalues() {
  if (!this.isSquare()) {
    throw new Error(INVALID_SQUARE_MATRIX);
  }

  if (this._eigenvalues !== undefined) {
    return this._eigenvalues;
  }

  var size = this.size()[0];
  var values = [];
  var digit = this._digit;
  var EPSILON = 1 / (Math.pow(10, digit) * 2);

  var clone = Matrix.clone(this)._matrix;

  var isConvergent = true; // flag

  var skip = false; // Transform matrix to Hessenberg matrix

  HouseholderTransform(clone, digit);

  for (var i = size - 1; i > 0; i--) {
    var divergenceCount = 0;
    var prev = void 0; // used to determine convergence
    // if obtains complex eigenvalues pair in previous iteration, skip current round

    if (skip) {
      skip = false;
      continue;
    }

    var shift = clone[size - 1][size - 1]; // eslint-disable-next-line no-constant-condition

    while (true) {
      if (!isConvergent) {
        // if the current eigenvalue is not real
        prev = size2Eigenvalues(clone[i - 1][i - 1], clone[i - 1][i], clone[i][i - 1], clone[i][i]).metric;
      } else {
        // if the current eigenvalue is real
        prev = Math.abs(clone[i][i - 1]);
      } // apply single shift


      for (var j = 0; j < size; j++) {
        clone[j][j] -= shift;
      } // Apply QR Algorithm


      HessenbergQR(clone, digit);

      for (var _j = 0; _j < size; _j++) {
        clone[_j][_j] += shift;
      }

      if (isConvergent && prev < Math.abs(clone[i][i - 1])) {
        divergenceCount++;
      } // if the current eigenvalue is real and the entry is almost ZERO => break;


      if (isConvergent && Math.abs(clone[i][i - 1]) < EPSILON) {
        values[i] = new Complex(clone[i][i]);
        break;
      } // if the current eigenvalues pair is complex, if the difference of the previous eiganvalues and the
      // eigenvalues of submatrix is almost ZERO => break


      var _size2Eigenvalues = size2Eigenvalues(clone[i - 1][i - 1], clone[i - 1][i], clone[i][i - 1], clone[i][i]),
          metric = _size2Eigenvalues.metric,
          eigen1 = _size2Eigenvalues.eigen1,
          eigen2 = _size2Eigenvalues.eigen2;

      if (!isConvergent && Math.abs(prev - metric) < EPSILON) {
        isConvergent = true; // re-initialize

        skip = true;
        var re1 = eigen1.re,
            im1 = eigen1.im;
        var re2 = eigen2.re,
            im2 = eigen2.im;
        values[i] = new Complex(re1, im1);
        values[i - 1] = new Complex(re2, im2);
        break;
      } // if the entry doesn't converge => complex eigenvalues pair


      if (divergenceCount > 3) {
        isConvergent = false;
      }
    }
  }

  if (!skip) {
    values[0] = new Complex(clone[0][0]);
  }

  this._eigenvalues = values;
  return values;
}

;

function HouseholderTransform(A, digit) {
  var size = A.length;
  var EPSILON = 1 / (Math.pow(10, digit) * 2);

  for (var j = 0; j < size - 2; j++) {
    var xNorm = 0;
    var u = new Array(size - j - 1);

    for (var i = j + 1; i < size; i++) {
      var entry = A[i][j];
      xNorm += Math.pow(entry, 2);
      u[i - j - 1] = entry;
    }

    xNorm = Math.sqrt(xNorm);

    if (Math.abs(xNorm) < EPSILON) {
      continue;
    }

    if (u[0] >= 0) {
      u[0] += xNorm;
    } else {
      u[0] -= xNorm;
    } // Make 'u' unit vector


    var uNorm = 0;

    for (var _i = 0; _i < u.length; _i++) {
      uNorm += Math.pow(u[_i], 2);
    }

    uNorm = Math.sqrt(uNorm);

    for (var _i2 = 0; _i2 < u.length; _i2++) {
      u[_i2] /= uNorm;
    } // update the matrix, multiply P from left


    for (var n = j; n < size; n++) {
      // column
      var v = new Array(size - j - 1);

      for (var m = j + 1; m < size; m++) {
        v[m - j - 1] = A[m][n];
      }

      var scaler = 0;

      for (var _m = 0; _m < v.length; _m++) {
        scaler += v[_m] * u[_m];
      }

      scaler *= 2;

      for (var _m2 = j + 1; _m2 < size; _m2++) {
        // row
        if (n === j && _m2 !== j + 1) {
          A[_m2][n] = 0;
        } else {
          A[_m2][n] = v[_m2 - j - 1] - scaler * u[_m2 - j - 1];
        }
      }
    } // update the matrix, multiply P from right


    for (var _m3 = 0; _m3 < size; _m3++) {
      // row
      var _v = new Array(size - j - 1);

      for (var _n = j + 1; _n < size; _n++) {
        _v[_n - j - 1] = A[_m3][_n];
      }

      var _scaler = 0;

      for (var _n2 = 0; _n2 < _v.length; _n2++) {
        _scaler += _v[_n2] * u[_n2];
      }

      _scaler *= 2;

      for (var _n3 = j + 1; _n3 < size; _n3++) {
        // column
        A[_m3][_n3] = _v[_n3 - j - 1] - _scaler * u[_n3 - j - 1];
      }
    }
  }
}

function HessenbergQR(H, digit) {
  var size = H.length;
  var EPSILON = 1 / (Math.pow(10, digit) * 2);
  var sincos = new Array(size - 1);

  for (var i = 0; i < size - 1; i++) {
    var a = H[i][i];
    var c = H[i + 1][i];
    var norm = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));

    if (norm < EPSILON) {
      continue;
    }

    var cos = a / norm;
    var sin = c * -1 / norm;
    sincos[i] = [sin, cos];
    var row1 = new Array(size - i);
    var row2 = new Array(size - i);

    for (var j = i; j < size; j++) {
      row1[j - i] = H[i][j];
      row2[j - i] = H[i + 1][j];
    }

    for (var _j2 = i; _j2 < size; _j2++) {
      H[i][_j2] = cos * row1[_j2 - i] + sin * -1 * row2[_j2 - i];

      if (i === _j2) {
        H[i + 1][_j2] = 0;
      } else {
        H[i + 1][_j2] = sin * row1[_j2 - i] + cos * row2[_j2 - i];
      }
    }
  }

  for (var _j3 = 0; _j3 < size - 1; _j3++) {
    if (!sincos[_j3]) {
      continue;
    }

    var _sincos$_j = _slicedToArray(sincos[_j3], 2),
        _sin = _sincos$_j[0],
        _cos = _sincos$_j[1];

    var col1 = new Array(_j3 + 2);
    var col2 = new Array(_j3 + 2);

    for (var _i3 = 0; _i3 <= _j3 + 1; _i3++) {
      col1[_i3] = H[_i3][_j3];
      col2[_i3] = H[_i3][_j3 + 1];
    }

    for (var _i4 = 0; _i4 <= _j3 + 1; _i4++) {
      H[_i4][_j3] = col1[_i4] * _cos - col2[_i4] * _sin;
      H[_i4][_j3 + 1] = col1[_i4] * _sin + col2[_i4] * _cos;
    }
  }
} // find the eigenvalues of 2x2 matrix


function size2Eigenvalues(e11, e12, e21, e22) {
  var b = (e11 + e22) * -1;
  var c = e11 * e22 - e21 * e12;
  var delta = Math.pow(b, 2) - 4 * c;
  var re1;
  var im1;
  var re2;
  var im2;

  if (delta >= 0) {
    im1 = 0;
    im2 = 0;

    if (b >= 0) {
      re1 = (b * -1 - Math.sqrt(delta)) / 2;
    } else {
      re1 = (b * -1 + Math.sqrt(delta)) / 2;
    }

    re2 = c / re1;
  } else {
    re1 = -b / 2;
    re2 = re1;
    im1 = Math.sqrt(delta * -1) / 2;
    im2 = im1 * -1;
  }

  return {
    metric: Math.sqrt(Math.pow(re1, 2) + Math.pow(im1, 2)),
    eigen1: {
      re: re1,
      im: im1
    },
    eigen2: {
      re: re2,
      im: im2
    }
  };
}

module.exports = eigenvalues;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/properties/norm.js":
/*!*******************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/properties/norm.js ***!
  \*******************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var Matrix = __webpack_require__(/*! ../.. */ "./node_modules/@rayyamhk/matrix/lib/index.js");

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_P_NORM = _require.INVALID_P_NORM;
/**
 * Calculates the Matrix norm of any Matrix with respect to the choice of norm.<br><br>
 * 
 * 1-norm: Maximum absolute column sum of the Matrix.<br>
 * 2-norm: The largest singular value of Matrix.<br>
 * Infinity-norm: Maximum absolute row sum of the Matrix.<br>
 * Frobenius-norm: Euclidean norm invloving all entries.<br><br>
 * 
 * The norms are not cached.
 * @memberof Matrix
 * @instance
 * @param {(1|2|Infinity|'F')} p - The choice of Matrix norm
 * @returns {number} The norm of the Matrix.
 */


function norm() {
  var p = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : 2;

  var _this$size = this.size(),
      _this$size2 = _slicedToArray(_this$size, 2),
      row = _this$size2[0],
      col = _this$size2[1];

  if (p !== 1 && p !== 2 && p !== Infinity && p !== 'F') {
    throw new Error(INVALID_P_NORM);
  }

  var matrix = this._matrix;
  var result = 0;

  if (p === 1) {
    // max of column sum
    for (var j = 0; j < col; j++) {
      var columnSum = 0;

      for (var i = 0; i < row; i++) {
        columnSum += Math.abs(matrix[i][j]);
      }

      if (columnSum > result) {
        result = columnSum;
      }
    }

    return result;
  } // largest singular value


  if (p === 2) {
    var transpose = Matrix.transpose(this);
    var M = Matrix.multiply(transpose, this);
    var eigenvalues = M.eigenvalues();

    for (var _i2 = 0; _i2 < eigenvalues.length; _i2++) {
      var value = eigenvalues[_i2].getModulus();

      if (value > result) {
        result = value;
      }
    }

    return Math.sqrt(result);
  }

  if (p === Infinity) {
    // max of row sum
    for (var _i3 = 0; _i3 < row; _i3++) {
      var rowSum = 0;

      for (var _j = 0; _j < col; _j++) {
        rowSum += Math.abs(matrix[_i3][_j]);
      }

      if (rowSum > result) {
        result = rowSum;
      }
    }

    return result;
  } // F


  for (var _i4 = 0; _i4 < row; _i4++) {
    for (var _j2 = 0; _j2 < col; _j2++) {
      result += Math.pow(matrix[_i4][_j2], 2);
    }
  }

  return Math.sqrt(result);
}

;
module.exports = norm;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/properties/nullity.js":
/*!**********************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/properties/nullity.js ***!
  \**********************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the nullity of any Matrix, which is the dimension
 * of the nullspace.<br><br>
 * 
 * The nullity is cached.
 * @memberof Matrix
 * @instance
 * @returns {number} The nullity of the matrix
 */
function nullity() {
  if (this._nullity !== undefined) {
    return this._nullity;
  }

  var col = this.size()[1];
  var rank = this.rank();
  this._nullity = col - rank;
  return this._nullity;
}

;
module.exports = nullity;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/properties/rank.js":
/*!*******************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/properties/rank.js ***!
  \*******************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var Matrix = __webpack_require__(/*! ../.. */ "./node_modules/@rayyamhk/matrix/lib/index.js");
/**
 * Calculates the rank of any Matrix,
 * which is the dimension of the row space.<br><br>
 * 
 * The rank is cached.
 * @memberof Matrix
 * @instance
 * @returns {number} The rank of the Matrix
 */


function rank() {
  if (this._rank !== undefined) {
    return this._rank;
  }

  var EPSILON = 1 / (Math.pow(10, this._digit) * 2);
  var R = Matrix.QR(this)[1];
  var matrixR = R._matrix;

  var _R$size = R.size(),
      _R$size2 = _slicedToArray(_R$size, 2),
      row = _R$size2[0],
      col = _R$size2[1];

  if (row === 0) {
    this._rank = 1;
    return 1;
  }

  var rk = 0;

  for (var i = 0; i < row; i++) {
    for (var j = i; j < col; j++) {
      if (Math.abs(matrixR[i][j]) >= EPSILON) {
        rk++;
        break;
      }
    }
  }

  this._rank = rk;
  return rk;
}

;
module.exports = rank;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/properties/size.js":
/*!*******************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/properties/size.js ***!
  \*******************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Calculates the size of any Matrix,
 * which is in the form of [row, column].<br><br>
 * 
 * The size of Matrix is cached.
 * @memberof Matrix
 * @instance
 * @returns {number[]} The number of rows and columns of a Matrix
 */
function size() {
  if (this._size !== undefined) {
    return this._size;
  }

  var A = this._matrix;

  if (A.length === 0) {
    this._size = [0, 0];
    return this._size;
  }

  this._size = [A.length, A[0].length];
  return this._size;
}

;
module.exports = size;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/properties/trace.js":
/*!********************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/properties/trace.js ***!
  \********************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_SQUARE_MATRIX = _require.INVALID_SQUARE_MATRIX;
/**
 * Calculates the trace of any square Matrix,
 * which is the sum of all entries on the main diagonal.<br><br>
 * 
 * The trace is cached.
 * @memberof Matrix
 * @instance
 * @returns {number} The trace of the square Matrix.
 */


function trace() {
  var isSquare = this._isSquare !== undefined ? this._isSquare : this.isSquare();

  if (!isSquare) {
    throw new Error(INVALID_SQUARE_MATRIX);
  }

  if (this._trace !== undefined) {
    return this._trace;
  }

  var A = this._matrix;
  var size = A.length;
  var tr = 0;

  for (var i = 0; i < size; i++) {
    tr += A[i][i];
  }

  this._trace = tr;
  return tr;
}

;
module.exports = trace;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/structure/isDiagonal.js":
/*!************************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/structure/isDiagonal.js ***!
  \************************************************************************/
/***/ ((module) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

/**
 * Determines whether a Matrix is diagonal or not.<br><br>
 * 
 * Diagonal Matrix is a Matrix in which the entries outside the main diagonal
 * are all zero. Note that the term diagonal refers to rectangular diagonal.<br><br>
 * 
 * The result is cached.
 * @memberof Matrix
 * @instance
 * @param {number} [digit=8] - Number of significant digits
 * @returns {boolean} Returns rue if the Matrix is diagonal Matrix
 */
function isDiagonal() {
  var digit = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : this._digit;

  if (this._isDiagonal !== undefined) {
    return this._isDiagonal;
  }

  var EPSILON = 1 / (Math.pow(10, digit) * 2);
  var A = this._matrix;

  var _this$size = this.size(),
      _this$size2 = _slicedToArray(_this$size, 2),
      row = _this$size2[0],
      col = _this$size2[1];

  if (row === 0) {
    this._isDiagonal = true;
    return true;
  }

  for (var i = 0; i < row; i++) {
    for (var j = 0; j < col; j++) {
      if (i !== j && Math.abs(A[i][j]) >= EPSILON) {
        this.isDiagonal = false;
        return false;
      }
    }
  }

  this._isDiagonal = true;
  return true;
}

;
module.exports = isDiagonal;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/structure/isLowerTriangular.js":
/*!*******************************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/structure/isLowerTriangular.js ***!
  \*******************************************************************************/
/***/ ((module) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

/**
 * Determines whether a Matrix is lower triangular Matrix or not.<br><br>
 * 
 * Lower triangular Matrix is a Matrix in which all the entries
 * above the main diagonal are zero. Note that it can be applied
 * to any non-square Matrix.<br><br>
 * 
 * The result is cached.
 * @memberof Matrix
 * @instance
 * @param {number} [digit=8] - Number of significant digits
 * @returns {boolean} Returns true if the Matrix is lower triangular
 */
function isLowerTriangular() {
  var digit = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : this._digit;

  if (this._isLowerTriangular !== undefined) {
    return this._isLowerTriangular;
  }

  var EPSILON = 1 / (Math.pow(10, digit) * 2);
  var A = this._matrix;

  var _this$size = this.size(),
      _this$size2 = _slicedToArray(_this$size, 2),
      row = _this$size2[0],
      col = _this$size2[1];

  if (row === 0) {
    // []
    this._isLowerTriangular = true;
    return true;
  }

  for (var i = 0; i < row; i++) {
    for (var j = i + 1; j < col; j++) {
      if (Math.abs(A[i][j]) >= EPSILON) {
        this._isLowerTriangular = false;
        return false;
      }
    }
  }

  this._isLowerTriangular = true;
  return true;
}

;
module.exports = isLowerTriangular;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/structure/isOrthogonal.js":
/*!**************************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/structure/isOrthogonal.js ***!
  \**************************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Determines whether a square Matrix is orthogonal or not.<br><br>
 * 
 * Orthogonal Matrix is a Matrix in which all rows and columns are
 * orthonormal vectors.<br><br>
 * 
 * The result is cached.
 * @memberof Matrix
 * @instance
 * @param {number} [digit=8] - Number of significant digits
 * @returns {boolean} Returns true if the square Matrix is orthogonal
 */
function isOrthogonal() {
  var digit = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : this._digit;

  if (this._isOrthogonal !== undefined) {
    return this._isOrthogonal;
  }

  if (!this.isSquare()) {
    this._isOrthogonal = false;
    return false;
  }

  var A = this._matrix;
  var EPSILON = 1 / (Math.pow(10, digit) * 2);
  var size = A.length;

  for (var i = 0; i < size; i++) {
    for (var j = i; j < size; j++) {
      var entry = 0;

      for (var k = 0; k < size; k++) {
        entry += A[i][k] * A[j][k];
      }

      if (i === j && Math.abs(entry - 1) >= EPSILON) {
        this._isOrthogonal = false;
        return false;
      }

      if (i !== j && Math.abs(entry) >= EPSILON) {
        this._isOrthogonal = false;
        return false;
      }
    }
  }

  this._isOrthogonal = true;
  return true;
}

;
module.exports = isOrthogonal;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/structure/isSkewSymmetric.js":
/*!*****************************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/structure/isSkewSymmetric.js ***!
  \*****************************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Determines whether a square Matrix is skew symmetric or not.<br><br>
 * 
 * Skew symmetric Matrix is a square Matrix whose transpose equals its negative.<br><br>
 * 
 * The result is cached.
 * @memberof Matrix
 * @instance
 * @param {number} [digit=8] - Number of significant digits
 * @returns {boolean} Returns true if the square Matrix is skew symmetric
 */
function isSkewSymmetric() {
  var digit = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : this._digit;

  if (this._isSkewSymmetric !== undefined) {
    return this._isSkewSymmetric;
  }

  if (!this.isSquare()) {
    this._isSkewSymmetric = false;
    return false;
  }

  var A = this._matrix;
  var EPSILON = 1 / (Math.pow(10, digit) * 2);
  var size = A.length;

  if (size === 0) {
    this._isSkewSymmetric = true;
    return true; // []
  }

  for (var i = 0; i < size; i++) {
    for (var j = 0; j < i; j++) {
      if (Math.abs(A[i][j] + A[j][i]) >= EPSILON) {
        this._isSkewSymmetric = false;
        return false;
      }
    }
  }

  this._isSkewSymmetric = true;
  return true;
}

;
module.exports = isSkewSymmetric;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/structure/isSquare.js":
/*!**********************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/structure/isSquare.js ***!
  \**********************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Determines whether a Matrix is square or not.<br><br>
 * 
 * Square Matrix is a Matrix with same number of rows and columns.<br><br>
 * 
 * The result is cached.
 * @memberof Matrix
 * @instance
 * @returns {boolean} Returns true if the Matrix is square
 */
function isSquare() {
  if (this._isSquare !== undefined) {
    return this._isSquare;
  }

  var A = this._matrix;

  if (A.length === 0) {
    // 0x0 matrix
    this._isSquare = true;
    return true;
  }

  this._isSquare = A.length === A[0].length;
  return this._isSquare;
}

;
module.exports = isSquare;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/structure/isSymmetric.js":
/*!*************************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/structure/isSymmetric.js ***!
  \*************************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Determines whether a square Matrix is symmetric or not.<br><br>
 * 
 * Symmetric Matrix is a square Matrix that is equal to its transpose.<br><br>
 * 
 * The result is cached.
 * @memberof Matrix
 * @instance
 * @param {number} [digit=8] - Number of significant digits
 * @returns {boolean} Returns true if the square Matrix is symmetric
 */
function isSymmetric() {
  var digit = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : this._digit;

  if (this._isSymmetric !== undefined) {
    return this._isSymmetric;
  }

  if (!this.isSquare()) {
    return false;
  }

  var A = this._matrix;
  var EPSILON = 1 / (Math.pow(10, digit) * 2);
  var size = A.length;

  for (var i = 0; i < size; i++) {
    for (var j = 0; j <= i; j++) {
      if (Math.abs(A[i][j] - A[j][i]) >= EPSILON) {
        this._isSymmetric = false;
        return false;
      }
    }
  }

  this._isSymmetric = true;
  return true;
}

;
module.exports = isSymmetric;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/structure/isUpperTriangular.js":
/*!*******************************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/structure/isUpperTriangular.js ***!
  \*******************************************************************************/
/***/ ((module) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

/**
 * Determines whether a Matrix is upper triangular Matrix or not.<br><br>
 * 
 * Upper triangular Matrix is a Matrix in which all the entries below the
 * main diagonal are zero. Note that it can be applied to any non-square Matrix.<br><br>
 *  
 * The result is cached.
 * @memberof Matrix
 * @instance
 * @param {number} [digit=8] - Number of significant digits
 * @returns {boolean} Returns true if the Matrix is upper triangular
 */
function isUpperTriangular() {
  var digit = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : this._digit;

  if (this._isUpperTriangular !== undefined) {
    return this._isUpperTriangular;
  }

  var EPSILON = 1 / (Math.pow(10, digit) * 2);
  var A = this._matrix;

  var _this$size = this.size(),
      _this$size2 = _slicedToArray(_this$size, 2),
      row = _this$size2[0],
      col = _this$size2[1];

  if (row === 0) {
    // []
    this._isUpperTriangular = true;
    return true;
  }

  for (var i = 0; i < row; i++) {
    for (var j = 0; j < col; j++) {
      if (i <= j) {
        continue;
      }

      if (Math.abs(A[i][j]) >= EPSILON) {
        this._isUpperTriangular = false;
        return false;
      }
    }
  }

  this._isUpperTriangular = true;
  return true;
}

;
module.exports = isUpperTriangular;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/utils/clone.js":
/*!***************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/utils/clone.js ***!
  \***************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX;
/**
 * Creates a copy of Matrix. Note that it resets the cached data.
 * @memberof Matrix
 * @static
 * @param {Matrix} A - Any Matrix
 * @returns {Matrix} Copy of A
 */


function clone(A) {
  if (!(A instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  var _A$size = A.size(),
      _A$size2 = _slicedToArray(_A$size, 2),
      row = _A$size2[0],
      col = _A$size2[1];

  var matrix = A._matrix;
  return this.generate(row, col, function (i, j) {
    return matrix[i][j];
  });
}

;
module.exports = clone;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/utils/column.js":
/*!****************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/utils/column.js ***!
  \****************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_ROW_COL = _require.INVALID_ROW_COL,
    OVERFLOW_COLUMN = _require.OVERFLOW_COLUMN,
    INVALID_MATRIX = _require.INVALID_MATRIX;
/**
 * Gets the column of a Matrix with valid index.
 * @memberof Matrix
 * @static
 * @param {Matrix} A - Any Matrix
 * @param {number} index - Any valid column index
 * @returns {Matrix} Column of A
 */


function column(A, index) {
  if (!(A instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  if (!Number.isInteger(index) || index < 0) {
    throw new Error(INVALID_ROW_COL);
  }

  var _A$size = A.size(),
      _A$size2 = _slicedToArray(_A$size, 2),
      r = _A$size2[0],
      c = _A$size2[1];

  if (index >= c) {
    throw new Error(OVERFLOW_COLUMN);
  }

  var matrix = A._matrix;
  return this.generate(r, 1, function (i) {
    return matrix[i][index];
  });
}

;
module.exports = column;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/utils/diag.js":
/*!**************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/utils/diag.js ***!
  \**************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


var Matrix = __webpack_require__(/*! ../.. */ "./node_modules/@rayyamhk/matrix/lib/index.js");

var isNumber = __webpack_require__(/*! ../../util/isNumber */ "./node_modules/@rayyamhk/matrix/lib/util/isNumber.js");

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_ARRAY = _require.INVALID_ARRAY,
    EXPECTED_ARRAY_OF_NUMBERS_OR_MATRICES = _require.EXPECTED_ARRAY_OF_NUMBERS_OR_MATRICES,
    INVALID_SQUARE_MATRIX = _require.INVALID_SQUARE_MATRIX;
/**
 * Generates diagonal Matrix if the argument is an array of numbers,
 * generates block diagonal Matrix if the argument is an array of Matrices.
 * @memberof Matrix
 * @static
 * @param {(number[]|Matrix[])} values - Array of numbers or Matrices
 * @returns {Matrix} Block diagonal Matrix
 */


function diag(values) {
  if (!Array.isArray(values)) {
    throw new Error(INVALID_ARRAY);
  }

  var argsNum = values.length;
  var variant;

  for (var i = 0; i < argsNum; i++) {
    var entry = values[i];

    if (!isNumber(entry) && !(entry instanceof Matrix)) {
      throw new Error(EXPECTED_ARRAY_OF_NUMBERS_OR_MATRICES);
    }

    if (isNumber(entry)) {
      if (!variant) {
        variant = 'number';
        continue;
      }

      if (variant !== 'number') {
        throw new Error(EXPECTED_ARRAY_OF_NUMBERS_OR_MATRICES);
      }
    } else {
      if (!entry.isSquare()) {
        throw new Error(INVALID_SQUARE_MATRIX);
      }

      if (!variant) {
        variant = 'square';
        continue;
      }

      if (variant !== 'square') {
        throw new Error(EXPECTED_ARRAY_OF_NUMBERS_OR_MATRICES);
      }
    }
  } // HERE: variant should be either 'number' or 'square'


  if (variant === 'number') {
    return Matrix.generate(argsNum, argsNum, function (i, j) {
      if (i === j) {
        return values[i];
      }

      return 0;
    });
  } // Guaranteed that [values] is a list of square matrices


  var size = 0;
  var temp = new Array(argsNum);

  for (var _i = 0; _i < argsNum; _i++) {
    var _len = values[_i].size()[0];

    size += _len;
    temp[_i] = _len;
  }

  var idx = 0;
  var start = 0;
  var len = temp[idx];
  return Matrix.generate(size, size, function (i, j) {
    if (i - start === len && j - start === len) {
      start += len;
      idx++;
    }

    var ith = i - start; // ith < 0 if below main diagonal

    var jth = j - start; // jth < 0 if above main diagonal
    // skip 0x0 matrices

    len = temp[idx];

    while (len === 0) {
      idx++;
      len = temp[idx];
    }

    if (ith < len && ith >= 0 && jth < len && jth >= 0) {
      return values[idx]._matrix[ith][jth];
    }

    return 0;
  });
}

;
module.exports = diag;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/utils/elementwise.js":
/*!*********************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/utils/elementwise.js ***!
  \*********************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX;
/**
 * This callback applies on each entry of a Matrix
 * @callback entryCallback
 * @param {number} entry - Entry of a Matrix
 * @returns {number} New entry value
 */

/**
 * Applys a function over each entry of a Matrix and returns
 * a new copy of the new Matrix.
 * @memberof Matrix
 * @static
 * @param {Matrix} A - Any Matrix
 * @param {entryCallback} cb - Callback function which applies on each entry of A
 * @returns {Matrix} A copy of new Matrix
 */


function elementwise(A, cb) {
  if (!(A instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  var _A$size = A.size(),
      _A$size2 = _slicedToArray(_A$size, 2),
      row = _A$size2[0],
      col = _A$size2[1];

  var matrix = A._matrix;
  return this.generate(row, col, function (i, j) {
    return cb(matrix[i][j]);
  });
}

;
module.exports = elementwise;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/utils/entry.js":
/*!***************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/utils/entry.js ***!
  \***************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_ROW_COL = _require.INVALID_ROW_COL,
    OVERFLOW_INDEX = _require.OVERFLOW_INDEX;
/**
 * Gets the entry of a Matrix.
 * @memberof Matrix
 * @instance
 * @param {number} row - Any valid row index
 * @param {number} col - Any valid column index
 * @returns {number} Entry of the Matrix
 */


function entry(row, col) {
  if (!Number.isInteger(row) || row < 0 || !Number.isInteger(col) || col < 0) {
    throw new Error(INVALID_ROW_COL);
  }

  var A = this._matrix;

  var _this$size = this.size(),
      _this$size2 = _slicedToArray(_this$size, 2),
      r = _this$size2[0],
      c = _this$size2[1];

  if (row >= r || col >= c) {
    throw new Error(OVERFLOW_INDEX);
  }

  return A[row][col];
}

;
module.exports = entry;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/utils/generate.js":
/*!******************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/utils/generate.js ***!
  \******************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


var empty = __webpack_require__(/*! ../../util/empty */ "./node_modules/@rayyamhk/matrix/lib/util/empty.js");
/**
 * This callback generates each entry of a Matrix
 * @callback generateCallback
 * @param {number} i - The i-th row of Matrix 
 * @param {number} j - The j-th column of Matrix 
 * @returns {number} Entry of Matrix
 */

/**
 * Generates a Matrix which entries are the returned value of callback function.
 * @memberof Matrix
 * @static
 * @param {number} row - Number of rows of Matrix
 * @param {number} col - Number of columns of Matrix
 * @param {generateCallback} cb - Callback function which takes row and column as arguments
 * and generates the corresponding entry
 * @returns {Matrix} - Generated Matrix
 */


function generate(row, col, cb) {
  var matrix = empty(row, col);

  if (row === 0 || col === 0) {
    return new this([]);
  }

  for (var i = 0; i < row; i++) {
    for (var j = 0; j < col; j++) {
      matrix[i][j] = cb(i, j);
    }
  }

  return new this(matrix);
}

;
module.exports = generate;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/utils/getDiag.js":
/*!*****************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/utils/getDiag.js ***!
  \*****************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX;
/**
 * Gets the entries on the main diagonal.
 * @memberof Matrix
 * @static
 * @param {Matrix} A - Any Matrix
 * @returns {number[]} Array of entries of A on the main diagonal
 */


function getDiag(A) {
  if (!(A instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  var _A$size = A.size(),
      _A$size2 = _slicedToArray(_A$size, 2),
      row = _A$size2[0],
      col = _A$size2[1];

  var size = Math.min(row, col);
  var matrix = A._matrix;
  var diags = new Array(size);

  for (var i = 0; i < size; i++) {
    diags[i] = matrix[i][i];
  }

  return diags;
}

;
module.exports = getDiag;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/utils/getRandomMatrix.js":
/*!*************************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/utils/getRandomMatrix.js ***!
  \*************************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Generates a random Matrix.
 * @memberof Matrix
 * @static
 * @param {number} row - Number of rows of a Matrix
 * @param {number} col - Number of columns of a Matrix
 * @param {number} min - Lower bound of each entry
 * @param {number} max - Upper bound of each entry
 * @param {number} toFixed - Number of decimal places
 * @returns {Matrix} Generated random Matrix
 */
function getRandomMatrix(row, col) {
  var min = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : 0;
  var max = arguments.length > 3 && arguments[3] !== undefined ? arguments[3] : 1;
  var toFixed = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : 0;
  return this.generate(row, col, function () {
    return Number.parseFloat((Math.random() * (max - min) + min).toFixed(toFixed));
  });
}

;
module.exports = getRandomMatrix;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/utils/identity.js":
/*!******************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/utils/identity.js ***!
  \******************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Generates identity Matrix with given size.
 * @memberof Matrix
 * @static
 * @param {number} size - The size of Matrix
 * @returns {Matrix} Identity Matrix
 */
function identity(size) {
  return this.generate(size, size, function (i, j) {
    if (i === j) {
      return 1;
    }

    return 0;
  });
}

;
module.exports = identity;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/utils/isEqual.js":
/*!*****************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/utils/isEqual.js ***!
  \*****************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX;
/**
 * Determines whether two Matrices are considered as equal.<br><br>
 * 
 * The test criterion is Math.abs(x - y) < 1 / (10 ** digit * 2).
 * For default value 5, it should be 5e-5.
 * That means if the difference of two numbers is less than 5e-5,
 * they are considered as same value.
 * @memberof Matrix
 * @static
 * @param {Matrix} A - Any Matrix
 * @param {Matrix} B - Any Matrix
 * @param {number} digit - Number of significant digits
 * @returns {boolean} Returns true if two Matrices are considered as same
 */


function isEqual(A, B) {
  var digit = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : 5;

  if (!(A instanceof this) || !(B instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  var _A$size = A.size(),
      _A$size2 = _slicedToArray(_A$size, 2),
      Arow = _A$size2[0],
      Acol = _A$size2[1];

  var _B$size = B.size(),
      _B$size2 = _slicedToArray(_B$size, 2),
      Brow = _B$size2[0],
      Bcol = _B$size2[1];

  if (Arow !== Brow || Acol !== Bcol) {
    return false;
  }

  var EPISILON = 1 / (Math.pow(10, digit) * 2);
  var matrixA = A._matrix;
  var matrixB = B._matrix;

  for (var i = 0; i < Arow; i++) {
    for (var j = 0; j < Acol; j++) {
      if (Math.abs(matrixA[i][j] - matrixB[i][j]) >= EPISILON) {
        return false;
      }
    }
  }

  return true;
}

;
module.exports = isEqual;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/utils/row.js":
/*!*************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/utils/row.js ***!
  \*************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_ROW_COL = _require.INVALID_ROW_COL,
    OVERFLOW_ROW = _require.OVERFLOW_ROW,
    INVALID_MATRIX = _require.INVALID_MATRIX;
/**
 * Gets the row of a Matrix with valid index.
 * @memberof Matrix
 * @static
 * @param {Matrix} A - Any Matrix
 * @param {number} index - Any valid row index
 * @returns {Matrix} Row of A
 */


function row(A, index) {
  if (!(A instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  if (!Number.isInteger(index) || index < 0) {
    throw new Error(INVALID_ROW_COL);
  }

  var _A$size = A.size(),
      _A$size2 = _slicedToArray(_A$size, 2),
      r = _A$size2[0],
      c = _A$size2[1];

  if (index >= r) {
    throw new Error(OVERFLOW_ROW);
  }

  var matrix = A._matrix;
  return this.generate(1, c, function (i, j) {
    return matrix[index][j];
  });
}

;
module.exports = row;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/utils/submatrix.js":
/*!*******************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/utils/submatrix.js ***!
  \*******************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

function _typeof(obj) { "@babel/helpers - typeof"; if (typeof Symbol === "function" && typeof Symbol.iterator === "symbol") { _typeof = function _typeof(obj) { return typeof obj; }; } else { _typeof = function _typeof(obj) { return obj && typeof Symbol === "function" && obj.constructor === Symbol && obj !== Symbol.prototype ? "symbol" : typeof obj; }; } return _typeof(obj); }

var _require = __webpack_require__(/*! ../../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX,
    EXPECTED_STRING_NUMBER_AT_POS_1_2 = _require.EXPECTED_STRING_NUMBER_AT_POS_1_2,
    INVALID_ROW = _require.INVALID_ROW,
    INVALID_COLUMN = _require.INVALID_COLUMN,
    OVERFLOW_ROW = _require.OVERFLOW_ROW,
    INVALID_ROWS_EXPRESSION = _require.INVALID_ROWS_EXPRESSION,
    INVALID_COLUMNS_EXPRESSION = _require.INVALID_COLUMNS_EXPRESSION,
    OVERFLOW_COLUMN = _require.OVERFLOW_COLUMN;
/**
 * Generates a submatrix of a matrix.
 * @memberof Matrix
 * @static
 * @param {Matrix} A - Any matrix
 * @param {string|number} rows - Rows expression
 * @param {string|number} cols - Columns expression
 * @returns {Matrix} Submatrix of A
 */


function submatrix(A, rows, cols) {
  if (!(A instanceof this)) {
    throw new Error(INVALID_MATRIX);
  }

  var arg1Type = _typeof(rows);

  var arg2Type = _typeof(cols);

  if (arg1Type !== 'string' && arg1Type !== 'number' || arg2Type !== 'string' && arg2Type !== 'number') {
    throw new Error(EXPECTED_STRING_NUMBER_AT_POS_1_2);
  }

  var _A$size = A.size(),
      _A$size2 = _slicedToArray(_A$size, 2),
      row = _A$size2[0],
      col = _A$size2[1];

  var rowStart;
  var rowEnd;
  var colStart;
  var colEnd;

  if (arg1Type === 'number') {
    if (!Number.isInteger(rows) || rows < 0) {
      throw new Error(INVALID_ROW);
    }

    if (rows >= row) {
      throw new Error(OVERFLOW_ROW);
    }

    rowStart = rows;
    rowEnd = rows;
  } else {
    // string
    var arg = rows.split(':');

    if (arg.length !== 2) {
      throw new Error(INVALID_ROWS_EXPRESSION);
    }

    var _arg = _slicedToArray(arg, 2),
        r1 = _arg[0],
        r2 = _arg[1];

    if (r1 === '') {
      rowStart = 0;
    } else {
      var r = Number(r1);

      if (!Number.isInteger(r) || r < 0) {
        throw new Error(INVALID_ROW);
      }

      if (r >= row) {
        throw new Error(OVERFLOW_ROW);
      }

      rowStart = r;
    }

    if (r2 === '') {
      rowEnd = row - 1;
    } else {
      var _r = Number(r2);

      if (!Number.isInteger(_r) || _r < 0) {
        throw new Error(INVALID_ROW);
      }

      if (_r >= row) {
        throw new Error(OVERFLOW_ROW);
      }

      rowEnd = _r;
    }

    if (rowStart > rowEnd) {
      throw new Error(INVALID_ROWS_EXPRESSION);
    }
  }

  if (arg2Type === 'number') {
    if (!Number.isInteger(cols) || cols < 0) {
      throw new Error(INVALID_COLUMN);
    }

    if (cols >= col) {
      throw new Error(OVERFLOW_COLUMN);
    }

    colStart = cols;
    colEnd = cols;
  } else {
    // string
    var _arg2 = cols.split(':');

    if (_arg2.length !== 2) {
      throw new Error(INVALID_COLUMNS_EXPRESSION);
    }

    var _arg3 = _slicedToArray(_arg2, 2),
        c1 = _arg3[0],
        c2 = _arg3[1];

    if (c1 === '') {
      colStart = 0;
    } else {
      var c = Number(c1);

      if (!Number.isInteger(c) || c < 0) {
        throw new Error(INVALID_COLUMN);
      }

      if (c >= col) {
        throw new Error(OVERFLOW_COLUMN);
      }

      colStart = c;
    }

    if (c2 === '') {
      colEnd = col - 1;
    } else {
      var _c = Number(c2);

      if (!Number.isInteger(_c) || _c < 0) {
        throw new Error(INVALID_COLUMN);
      }

      if (_c >= col) {
        throw new Error(OVERFLOW_COLUMN);
      }

      colEnd = _c;
    }

    if (colStart > colEnd) {
      throw new Error(INVALID_COLUMNS_EXPRESSION);
    }
  }

  var matrix = A._matrix;
  var subRow = rowEnd - rowStart + 1;
  var subCol = colEnd - colStart + 1;
  var subMatrix = new Array(subRow);

  for (var i = rowStart; i <= rowEnd; i++) {
    var newRow = new Array(subCol);

    for (var j = colStart; j <= colEnd; j++) {
      newRow[j - colStart] = matrix[i][j];
    }

    subMatrix[i - rowStart] = newRow;
  }

  return new this(subMatrix);
}

;
module.exports = submatrix;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/utils/toString.js":
/*!******************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/utils/toString.js ***!
  \******************************************************************/
/***/ ((module) => {

"use strict";


function _slicedToArray(arr, i) { return _arrayWithHoles(arr) || _iterableToArrayLimit(arr, i) || _unsupportedIterableToArray(arr, i) || _nonIterableRest(); }

function _nonIterableRest() { throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method."); }

function _unsupportedIterableToArray(o, minLen) { if (!o) return; if (typeof o === "string") return _arrayLikeToArray(o, minLen); var n = Object.prototype.toString.call(o).slice(8, -1); if (n === "Object" && o.constructor) n = o.constructor.name; if (n === "Map" || n === "Set") return Array.from(o); if (n === "Arguments" || /^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)) return _arrayLikeToArray(o, minLen); }

function _arrayLikeToArray(arr, len) { if (len == null || len > arr.length) len = arr.length; for (var i = 0, arr2 = new Array(len); i < len; i++) { arr2[i] = arr[i]; } return arr2; }

function _iterableToArrayLimit(arr, i) { if (typeof Symbol === "undefined" || !(Symbol.iterator in Object(arr))) return; var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"] != null) _i["return"](); } finally { if (_d) throw _e; } } return _arr; }

function _arrayWithHoles(arr) { if (Array.isArray(arr)) return arr; }

/**
 * Gets the stringified Matrix
 * @memberof Matrix
 * @instance
 * @returns {string} Stringified Matrix
 */
function toString() {
  var matrix = this._matrix;

  var _this$size = this.size(),
      _this$size2 = _slicedToArray(_this$size, 2),
      row = _this$size2[0],
      col = _this$size2[1];

  var str = '';

  for (var i = 0; i < row; i++) {
    for (var j = 0; j < col; j++) {
      str += matrix[i][j].toString();

      if (j !== col - 1) {
        str += ' ';
      }
    }

    if (i !== row - 1) {
      str += '\n';
    }
  }

  return str;
}

;
module.exports = toString;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/core/utils/zero.js":
/*!**************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/core/utils/zero.js ***!
  \**************************************************************/
/***/ ((module) => {

"use strict";


/**
 * Generates a zero Matrix
 * @memberof Matrix
 * @static
 * @param {number} row - Number of rows of the Matrix
 * @param {number} col - Number of columns of the Matrix
 * @returns {Matrix} Zero Matrix
 */
function zero(row, col) {
  if (col === undefined) {
    return this.generate(row, row, function () {
      return 0;
    });
  }

  return this.generate(row, col, function () {
    return 0;
  });
}

;
module.exports = zero;

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/index.js":
/*!****************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/index.js ***!
  \****************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


var isMatrix = __webpack_require__(/*! ./util/isMatrix */ "./node_modules/@rayyamhk/matrix/lib/util/isMatrix.js");

var _require = __webpack_require__(/*! ./Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_MATRIX = _require.INVALID_MATRIX;
/**
 * Creates a new Matrix
 * @namespace Matrix
 * @class
 * @param {number[][]} A - Two dimensional array where
 * A[i][j] represents the i-th row and j-th column of a matrix
 */


function Matrix(A) {
  if (!isMatrix(A)) {
    throw new Error(INVALID_MATRIX);
  }

  this._matrix = A;
  this._digit = 8;
}

module.exports = Matrix; // structure

Matrix.prototype.isDiagonal = __webpack_require__(/*! ./core/structure/isDiagonal */ "./node_modules/@rayyamhk/matrix/lib/core/structure/isDiagonal.js");
Matrix.prototype.isSkewSymmetric = __webpack_require__(/*! ./core/structure/isSkewSymmetric */ "./node_modules/@rayyamhk/matrix/lib/core/structure/isSkewSymmetric.js");
Matrix.prototype.isSquare = __webpack_require__(/*! ./core/structure/isSquare */ "./node_modules/@rayyamhk/matrix/lib/core/structure/isSquare.js");
Matrix.prototype.isSymmetric = __webpack_require__(/*! ./core/structure/isSymmetric */ "./node_modules/@rayyamhk/matrix/lib/core/structure/isSymmetric.js");
Matrix.prototype.isLowerTriangular = __webpack_require__(/*! ./core/structure/isLowerTriangular */ "./node_modules/@rayyamhk/matrix/lib/core/structure/isLowerTriangular.js");
Matrix.prototype.isUpperTriangular = __webpack_require__(/*! ./core/structure/isUpperTriangular */ "./node_modules/@rayyamhk/matrix/lib/core/structure/isUpperTriangular.js");
Matrix.prototype.isOrthogonal = __webpack_require__(/*! ./core/structure/isOrthogonal */ "./node_modules/@rayyamhk/matrix/lib/core/structure/isOrthogonal.js"); // property

Matrix.prototype.cond = __webpack_require__(/*! ./core/properties/cond */ "./node_modules/@rayyamhk/matrix/lib/core/properties/cond.js");
Matrix.prototype.det = __webpack_require__(/*! ./core/properties/det */ "./node_modules/@rayyamhk/matrix/lib/core/properties/det.js");
Matrix.prototype.eigenvalues = __webpack_require__(/*! ./core/properties/eigenvalues */ "./node_modules/@rayyamhk/matrix/lib/core/properties/eigenvalues.js");
Matrix.prototype.nullity = __webpack_require__(/*! ./core/properties/nullity */ "./node_modules/@rayyamhk/matrix/lib/core/properties/nullity.js");
Matrix.prototype.norm = __webpack_require__(/*! ./core/properties/norm */ "./node_modules/@rayyamhk/matrix/lib/core/properties/norm.js");
Matrix.prototype.rank = __webpack_require__(/*! ./core/properties/rank */ "./node_modules/@rayyamhk/matrix/lib/core/properties/rank.js");
Matrix.prototype.size = __webpack_require__(/*! ./core/properties/size */ "./node_modules/@rayyamhk/matrix/lib/core/properties/size.js");
Matrix.prototype.trace = __webpack_require__(/*! ./core/properties/trace */ "./node_modules/@rayyamhk/matrix/lib/core/properties/trace.js"); // operations

Matrix.add = __webpack_require__(/*! ./core/operations/add */ "./node_modules/@rayyamhk/matrix/lib/core/operations/add.js");
Matrix.inverse = __webpack_require__(/*! ./core/operations/inverse */ "./node_modules/@rayyamhk/matrix/lib/core/operations/inverse.js");
Matrix.multiply = __webpack_require__(/*! ./core/operations/multiply */ "./node_modules/@rayyamhk/matrix/lib/core/operations/multiply.js");
Matrix.pow = __webpack_require__(/*! ./core/operations/pow */ "./node_modules/@rayyamhk/matrix/lib/core/operations/pow.js");
Matrix.subtract = __webpack_require__(/*! ./core/operations/subtract */ "./node_modules/@rayyamhk/matrix/lib/core/operations/subtract.js");
Matrix.transpose = __webpack_require__(/*! ./core/operations/transpose */ "./node_modules/@rayyamhk/matrix/lib/core/operations/transpose.js"); // Linear-equations

Matrix.backward = __webpack_require__(/*! ./core/linear-equations/backward */ "./node_modules/@rayyamhk/matrix/lib/core/linear-equations/backward.js");
Matrix.forward = __webpack_require__(/*! ./core/linear-equations/forward */ "./node_modules/@rayyamhk/matrix/lib/core/linear-equations/forward.js");
Matrix.solve = __webpack_require__(/*! ./core/linear-equations/solve */ "./node_modules/@rayyamhk/matrix/lib/core/linear-equations/solve.js"); // decompositions

Matrix.LU = __webpack_require__(/*! ./core/decompositions/LU */ "./node_modules/@rayyamhk/matrix/lib/core/decompositions/LU.js");
Matrix.QR = __webpack_require__(/*! ./core/decompositions/QR */ "./node_modules/@rayyamhk/matrix/lib/core/decompositions/QR.js"); // utils

Matrix.clone = __webpack_require__(/*! ./core/utils/clone */ "./node_modules/@rayyamhk/matrix/lib/core/utils/clone.js");
Matrix.column = __webpack_require__(/*! ./core/utils/column */ "./node_modules/@rayyamhk/matrix/lib/core/utils/column.js");
Matrix.diag = __webpack_require__(/*! ./core/utils/diag */ "./node_modules/@rayyamhk/matrix/lib/core/utils/diag.js");
Matrix.elementwise = __webpack_require__(/*! ./core/utils/elementwise */ "./node_modules/@rayyamhk/matrix/lib/core/utils/elementwise.js");
Matrix.generate = __webpack_require__(/*! ./core/utils/generate */ "./node_modules/@rayyamhk/matrix/lib/core/utils/generate.js");
Matrix.getDiag = __webpack_require__(/*! ./core/utils/getDiag */ "./node_modules/@rayyamhk/matrix/lib/core/utils/getDiag.js");
Matrix.getRandomMatrix = __webpack_require__(/*! ./core/utils/getRandomMatrix */ "./node_modules/@rayyamhk/matrix/lib/core/utils/getRandomMatrix.js");
Matrix.identity = __webpack_require__(/*! ./core/utils/identity */ "./node_modules/@rayyamhk/matrix/lib/core/utils/identity.js");
Matrix.isEqual = __webpack_require__(/*! ./core/utils/isEqual */ "./node_modules/@rayyamhk/matrix/lib/core/utils/isEqual.js");
Matrix.row = __webpack_require__(/*! ./core/utils/row */ "./node_modules/@rayyamhk/matrix/lib/core/utils/row.js");
Matrix.submatrix = __webpack_require__(/*! ./core/utils/submatrix */ "./node_modules/@rayyamhk/matrix/lib/core/utils/submatrix.js");
Matrix.zero = __webpack_require__(/*! ./core/utils/zero */ "./node_modules/@rayyamhk/matrix/lib/core/utils/zero.js");
Matrix.prototype.entry = __webpack_require__(/*! ./core/utils/entry */ "./node_modules/@rayyamhk/matrix/lib/core/utils/entry.js");
Matrix.prototype.toString = __webpack_require__(/*! ./core/utils/toString */ "./node_modules/@rayyamhk/matrix/lib/core/utils/toString.js");

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/util/empty.js":
/*!*********************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/util/empty.js ***!
  \*********************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


var _require = __webpack_require__(/*! ../Error */ "./node_modules/@rayyamhk/matrix/lib/Error.js"),
    INVALID_ROW_COL = _require.INVALID_ROW_COL;

module.exports = function empty(row, col) {
  if (!Number.isInteger(row) || row < 0 || !Number.isInteger(col) || col < 0) {
    throw new Error(INVALID_ROW_COL);
  }

  if (row === 0 || col === 0) {
    return [];
  }

  var matrix = new Array(row);

  for (var i = 0; i < row; i++) {
    matrix[i] = new Array(col);
  }

  return matrix;
};

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/util/isMatrix.js":
/*!************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/util/isMatrix.js ***!
  \************************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

"use strict";


var isNumber = __webpack_require__(/*! ./isNumber */ "./node_modules/@rayyamhk/matrix/lib/util/isNumber.js");

module.exports = function isMatrix(matrix) {
  if (!Array.isArray(matrix)) {
    return false;
  }

  var height = matrix.length;

  if (height === 0) {
    return true; // [] represents empty matrix (0 x 0 matrix)
  }

  var firstRow = matrix[0];

  if (!Array.isArray(firstRow)) {
    return false;
  }

  var width = firstRow.length;

  if (width === 0) {
    return false; // [ [] ] is not allowed
  }

  for (var i = 0; i < height; i++) {
    var row = matrix[i];

    if (!Array.isArray(row) || row.length !== width) {
      return false;
    }

    for (var j = 0; j < width; j++) {
      if (!isNumber(row[j])) {
        return false;
      }
    }
  }

  return true;
};

/***/ }),

/***/ "./node_modules/@rayyamhk/matrix/lib/util/isNumber.js":
/*!************************************************************!*\
  !*** ./node_modules/@rayyamhk/matrix/lib/util/isNumber.js ***!
  \************************************************************/
/***/ ((module) => {

"use strict";


module.exports = function isNumber(_int) {
  return Number.isFinite(_int);
};

/***/ }),

/***/ "./node_modules/matrix-inverse/matrix-inverse.js":
/*!*******************************************************!*\
  !*** ./node_modules/matrix-inverse/matrix-inverse.js ***!
  \*******************************************************/
/***/ ((module) => {

var Sylvester = {}

Sylvester.Matrix = function () {}

Sylvester.Matrix.create = function (elements) {
  var M = new Sylvester.Matrix()
  return M.setElements(elements)
}

Sylvester.Matrix.I = function (n) {
  var els = [],
    i = n,
    j
  while (i--) {
    j = n
    els[i] = []
    while (j--) {
      els[i][j] = i === j ? 1 : 0
    }
  }
  return Sylvester.Matrix.create(els)
}

Sylvester.Matrix.prototype = {
  dup: function () {
    return Sylvester.Matrix.create(this.elements)
  },

  isSquare: function () {
    var cols = this.elements.length === 0 ? 0 : this.elements[0].length
    return this.elements.length === cols
  },

  toRightTriangular: function () {
    if (this.elements.length === 0) return Sylvester.Matrix.create([])
    var M = this.dup(),
      els
    var n = this.elements.length,
      i,
      j,
      np = this.elements[0].length,
      p
    for (i = 0; i < n; i++) {
      if (M.elements[i][i] === 0) {
        for (j = i + 1; j < n; j++) {
          if (M.elements[j][i] !== 0) {
            els = []
            for (p = 0; p < np; p++) {
              els.push(M.elements[i][p] + M.elements[j][p])
            }
            M.elements[i] = els
            break
          }
        }
      }
      if (M.elements[i][i] !== 0) {
        for (j = i + 1; j < n; j++) {
          var multiplier = M.elements[j][i] / M.elements[i][i]
          els = []
          for (p = 0; p < np; p++) {
            // Elements with column numbers up to an including the number of the
            // row that we're subtracting can safely be set straight to zero,
            // since that's the point of this routine and it avoids having to
            // loop over and correct rounding errors later
            els.push(
              p <= i ? 0 : M.elements[j][p] - M.elements[i][p] * multiplier
            )
          }
          M.elements[j] = els
        }
      }
    }
    return M
  },

  determinant: function () {
    if (this.elements.length === 0) {
      return 1
    }
    if (!this.isSquare()) {
      return null
    }
    var M = this.toRightTriangular()
    var det = M.elements[0][0],
      n = M.elements.length
    for (var i = 1; i < n; i++) {
      det = det * M.elements[i][i]
    }
    return det
  },

  isSingular: function () {
    return this.isSquare() && this.determinant() === 0
  },

  augment: function (matrix) {
    if (this.elements.length === 0) {
      return this.dup()
    }
    var M = matrix.elements || matrix
    if (typeof M[0][0] === 'undefined') {
      M = Sylvester.Matrix.create(M).elements
    }
    var T = this.dup(),
      cols = T.elements[0].length
    var i = T.elements.length,
      nj = M[0].length,
      j
    if (i !== M.length) {
      return null
    }
    while (i--) {
      j = nj
      while (j--) {
        T.elements[i][cols + j] = M[i][j]
      }
    }
    return T
  },

  inverse: function () {
    if (this.elements.length === 0) {
      return null
    }
    if (!this.isSquare() || this.isSingular()) {
      return null
    }
    var n = this.elements.length,
      i = n,
      j
    var M = this.augment(Sylvester.Matrix.I(n)).toRightTriangular()
    var np = M.elements[0].length,
      p,
      els,
      divisor
    var inverse_elements = [],
      new_element
    // Sylvester.Matrix is non-singular so there will be no zeros on the
    // diagonal. Cycle through rows from last to first.
    while (i--) {
      // First, normalise diagonal elements to 1
      els = []
      inverse_elements[i] = []
      divisor = M.elements[i][i]
      for (p = 0; p < np; p++) {
        new_element = M.elements[i][p] / divisor
        els.push(new_element)
        // Shuffle off the current row of the right hand side into the results
        // array as it will not be modified by later runs through this loop
        if (p >= n) {
          inverse_elements[i].push(new_element)
        }
      }
      M.elements[i] = els
      // Then, subtract this row from those above it to give the identity matrix
      // on the left hand side
      j = i
      while (j--) {
        els = []
        for (p = 0; p < np; p++) {
          els.push(M.elements[j][p] - M.elements[i][p] * M.elements[j][i])
        }
        M.elements[j] = els
      }
    }
    return Sylvester.Matrix.create(inverse_elements)
  },

  setElements: function (els) {
    var i,
      j,
      elements = els.elements || els
    if (elements[0] && typeof elements[0][0] !== 'undefined') {
      i = elements.length
      this.elements = []
      while (i--) {
        j = elements[i].length
        this.elements[i] = []
        while (j--) {
          this.elements[i][j] = elements[i][j]
        }
      }
      return this
    }
    var n = elements.length
    this.elements = []
    for (i = 0; i < n; i++) {
      this.elements.push([elements[i]])
    }
    return this
  },
}

module.exports = function (elements) {
  const mat = Sylvester.Matrix.create(elements).inverse()
  if (mat !== null) {
    return mat.elements
  } else {
    return null
  }
}


/***/ })

/******/ 	});
/************************************************************************/
/******/ 	// The module cache
/******/ 	var __webpack_module_cache__ = {};
/******/ 	
/******/ 	// The require function
/******/ 	function __webpack_require__(moduleId) {
/******/ 		// Check if module is in cache
/******/ 		if(__webpack_module_cache__[moduleId]) {
/******/ 			return __webpack_module_cache__[moduleId].exports;
/******/ 		}
/******/ 		// Create a new module (and put it into the cache)
/******/ 		var module = __webpack_module_cache__[moduleId] = {
/******/ 			// no module.id needed
/******/ 			// no module.loaded needed
/******/ 			exports: {}
/******/ 		};
/******/ 	
/******/ 		// Execute the module function
/******/ 		__webpack_modules__[moduleId](module, module.exports, __webpack_require__);
/******/ 	
/******/ 		// Return the exports of the module
/******/ 		return module.exports;
/******/ 	}
/******/ 	
/************************************************************************/
/******/ 	// module exports must be returned from runtime so entry inlining is disabled
/******/ 	// startup
/******/ 	// Load entry module and return exports
/******/ 	return __webpack_require__("./index.js");
/******/ })()
;
//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9pbmRleC5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9saWIvY29yZS1rYWxtYW4tZmlsdGVyLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi9keW5hbWljL2NvbnN0YW50LWFjY2VsZXJhdGlvbi5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9saWIvZHluYW1pYy9jb25zdGFudC1wb3NpdGlvbi5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9saWIvZHluYW1pYy9jb25zdGFudC1zcGVlZC5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9saWIva2FsbWFuLWZpbHRlci5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9saWIvbGluYWxnZWJyYS9hZGQuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbGliL2xpbmFsZ2VicmEvZGlhZy5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9saWIvbGluYWxnZWJyYS9kaXN0YW5jZS1tYXQuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbGliL2xpbmFsZ2VicmEvZWxlbS13aXNlLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi9saW5hbGdlYnJhL2lkZW50aXR5LmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi9saW5hbGdlYnJhL2luZGV4LmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi9saW5hbGdlYnJhL2ludmVydC5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9saWIvbGluYWxnZWJyYS9tYXQtbXVsLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi9saW5hbGdlYnJhL3BhZC13aXRoLXplcm9zLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi9saW5hbGdlYnJhL3N1Yi1zcXVhcmUtbWF0cml4LmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi9saW5hbGdlYnJhL3N1Yi5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9saWIvbGluYWxnZWJyYS9zdW0uanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbGliL2xpbmFsZ2VicmEvdHJhY2UuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbGliL2xpbmFsZ2VicmEvdHJhbnNwb3NlLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi9saW5hbGdlYnJhL3plcm9zLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi9tb2RlbC1jb2xsZWN0aW9uLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi9vYnNlcnZhdGlvbi9zZW5zb3IuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbGliL3NldHVwL2J1aWxkLXN0YXRlLXByb2plY3Rpb24uanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbGliL3NldHVwL2NoZWNrLWRpbWVuc2lvbnMuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbGliL3NldHVwL2V4dGVuZC1keW5hbWljLWluaXQuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbGliL3NldHVwL3NldC1kaW1lbnNpb25zLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi9zdGF0ZS5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9saWIvdXRpbHMvYXJyYXktdG8tbWF0cml4LmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi91dGlscy9jaGVjay1jb3ZhcmlhbmNlLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi91dGlscy9jaGVjay1tYXRyaXguanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbGliL3V0aWxzL2NoZWNrLXNoYXBlLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi91dGlscy9jb3JyZWxhdGlvbi10by1jb3ZhcmlhbmNlLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi91dGlscy9jb3ZhcmlhbmNlLXRvLWNvcnJlbGF0aW9uLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi91dGlscy9kZWVwLWFzc2lnbi5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9saWIvdXRpbHMvZ2V0LWNvdmFyaWFuY2UuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbGliL3V0aWxzL3BvbHltb3JwaC1tYXRyaXguanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbGliL3V0aWxzL3RvLWZ1bmN0aW9uLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL2xpYi91dGlscy91bmlxLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvY29tcGxleC9saWIvY29yZS9pbnN0YW5jZS9nZXRBcmd1bWVudC5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL2NvbXBsZXgvbGliL2NvcmUvaW5zdGFuY2UvZ2V0SW1hZ2luYXJ5LmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvY29tcGxleC9saWIvY29yZS9pbnN0YW5jZS9nZXRNb2R1bHVzLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvY29tcGxleC9saWIvY29yZS9pbnN0YW5jZS9nZXRSZWFsLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvY29tcGxleC9saWIvY29yZS9pbnN0YW5jZS90b1N0cmluZy5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL2NvbXBsZXgvbGliL2NvcmUvc3RhdGljL2Fjb3MuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9jb21wbGV4L2xpYi9jb3JlL3N0YXRpYy9hY290LmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvY29tcGxleC9saWIvY29yZS9zdGF0aWMvYWNzYy5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL2NvbXBsZXgvbGliL2NvcmUvc3RhdGljL2FkZC5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL2NvbXBsZXgvbGliL2NvcmUvc3RhdGljL2FzZWMuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9jb21wbGV4L2xpYi9jb3JlL3N0YXRpYy9hc2luLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvY29tcGxleC9saWIvY29yZS9zdGF0aWMvYXRhbi5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL2NvbXBsZXgvbGliL2NvcmUvc3RhdGljL2Nvbmp1Z2F0ZS5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL2NvbXBsZXgvbGliL2NvcmUvc3RhdGljL2Nvcy5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL2NvbXBsZXgvbGliL2NvcmUvc3RhdGljL2NvdC5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL2NvbXBsZXgvbGliL2NvcmUvc3RhdGljL2NzYy5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL2NvbXBsZXgvbGliL2NvcmUvc3RhdGljL2RpdmlkZS5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL2NvbXBsZXgvbGliL2NvcmUvc3RhdGljL2V4cC5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL2NvbXBsZXgvbGliL2NvcmUvc3RhdGljL2ludmVyc2UuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9jb21wbGV4L2xpYi9jb3JlL3N0YXRpYy9pc0VxdWFsLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvY29tcGxleC9saWIvY29yZS9zdGF0aWMvaXNOYU4uanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9jb21wbGV4L2xpYi9jb3JlL3N0YXRpYy9sb2cuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9jb21wbGV4L2xpYi9jb3JlL3N0YXRpYy9tdWx0aXBseS5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL2NvbXBsZXgvbGliL2NvcmUvc3RhdGljL3Bvdy5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL2NvbXBsZXgvbGliL2NvcmUvc3RhdGljL3NlYy5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL2NvbXBsZXgvbGliL2NvcmUvc3RhdGljL3Npbi5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL2NvbXBsZXgvbGliL2NvcmUvc3RhdGljL3N1YnRyYWN0LmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvY29tcGxleC9saWIvY29yZS9zdGF0aWMvdGFuLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvY29tcGxleC9saWIvaW5kZXguanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9tYXRyaXgvbGliL0Vycm9yLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL2RlY29tcG9zaXRpb25zL0xVLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL2RlY29tcG9zaXRpb25zL1FSLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL2xpbmVhci1lcXVhdGlvbnMvYmFja3dhcmQuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9tYXRyaXgvbGliL2NvcmUvbGluZWFyLWVxdWF0aW9ucy9mb3J3YXJkLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL2xpbmVhci1lcXVhdGlvbnMvc29sdmUuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9tYXRyaXgvbGliL2NvcmUvb3BlcmF0aW9ucy9hZGQuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9tYXRyaXgvbGliL2NvcmUvb3BlcmF0aW9ucy9pbnZlcnNlLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL29wZXJhdGlvbnMvbXVsdGlwbHkuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9tYXRyaXgvbGliL2NvcmUvb3BlcmF0aW9ucy9wb3cuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9tYXRyaXgvbGliL2NvcmUvb3BlcmF0aW9ucy9zdWJ0cmFjdC5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL21hdHJpeC9saWIvY29yZS9vcGVyYXRpb25zL3RyYW5zcG9zZS5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL21hdHJpeC9saWIvY29yZS9wcm9wZXJ0aWVzL2NvbmQuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9tYXRyaXgvbGliL2NvcmUvcHJvcGVydGllcy9kZXQuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9tYXRyaXgvbGliL2NvcmUvcHJvcGVydGllcy9laWdlbnZhbHVlcy5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL21hdHJpeC9saWIvY29yZS9wcm9wZXJ0aWVzL25vcm0uanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9tYXRyaXgvbGliL2NvcmUvcHJvcGVydGllcy9udWxsaXR5LmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL3Byb3BlcnRpZXMvcmFuay5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL21hdHJpeC9saWIvY29yZS9wcm9wZXJ0aWVzL3NpemUuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9tYXRyaXgvbGliL2NvcmUvcHJvcGVydGllcy90cmFjZS5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL21hdHJpeC9saWIvY29yZS9zdHJ1Y3R1cmUvaXNEaWFnb25hbC5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL21hdHJpeC9saWIvY29yZS9zdHJ1Y3R1cmUvaXNMb3dlclRyaWFuZ3VsYXIuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9tYXRyaXgvbGliL2NvcmUvc3RydWN0dXJlL2lzT3J0aG9nb25hbC5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL21hdHJpeC9saWIvY29yZS9zdHJ1Y3R1cmUvaXNTa2V3U3ltbWV0cmljLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL3N0cnVjdHVyZS9pc1NxdWFyZS5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL21hdHJpeC9saWIvY29yZS9zdHJ1Y3R1cmUvaXNTeW1tZXRyaWMuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9tYXRyaXgvbGliL2NvcmUvc3RydWN0dXJlL2lzVXBwZXJUcmlhbmd1bGFyLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL3V0aWxzL2Nsb25lLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL3V0aWxzL2NvbHVtbi5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL21hdHJpeC9saWIvY29yZS91dGlscy9kaWFnLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL3V0aWxzL2VsZW1lbnR3aXNlLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL3V0aWxzL2VudHJ5LmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL3V0aWxzL2dlbmVyYXRlLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL3V0aWxzL2dldERpYWcuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9tYXRyaXgvbGliL2NvcmUvdXRpbHMvZ2V0UmFuZG9tTWF0cml4LmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL3V0aWxzL2lkZW50aXR5LmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL3V0aWxzL2lzRXF1YWwuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyLy4vbm9kZV9tb2R1bGVzL0ByYXl5YW1oay9tYXRyaXgvbGliL2NvcmUvdXRpbHMvcm93LmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9jb3JlL3V0aWxzL3N1Ym1hdHJpeC5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL21hdHJpeC9saWIvY29yZS91dGlscy90b1N0cmluZy5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL21hdHJpeC9saWIvY29yZS91dGlscy96ZXJvLmpzIiwid2VicGFjazovL2thbG1hbkZpbHRlci8uL25vZGVfbW9kdWxlcy9AcmF5eWFtaGsvbWF0cml4L2xpYi9pbmRleC5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL21hdHJpeC9saWIvdXRpbC9lbXB0eS5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL21hdHJpeC9saWIvdXRpbC9pc01hdHJpeC5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvQHJheXlhbWhrL21hdHJpeC9saWIvdXRpbC9pc051bWJlci5qcyIsIndlYnBhY2s6Ly9rYWxtYW5GaWx0ZXIvLi9ub2RlX21vZHVsZXMvbWF0cml4LWludmVyc2UvbWF0cml4LWludmVyc2UuanMiLCJ3ZWJwYWNrOi8va2FsbWFuRmlsdGVyL3dlYnBhY2svYm9vdHN0cmFwIiwid2VicGFjazovL2thbG1hbkZpbHRlci93ZWJwYWNrL3N0YXJ0dXAiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6Ijs7Ozs7Ozs7OztBQUFBLHdCQUF3QixtQkFBTyxDQUFDLHlEQUF3Qjs7QUFFeEQ7QUFDQTtBQUNBLGVBQWUsbUJBQU8sQ0FBQyxtREFBcUI7QUFDNUM7QUFDQTtBQUNBO0FBQ0EsZ0JBQWdCLG1CQUFPLENBQUMsaUVBQTRCO0FBQ3BELFFBQVEsbUJBQU8sQ0FBQyxtQ0FBYTtBQUM3QixrQkFBa0IsbUJBQU8sQ0FBQyxxRUFBOEI7QUFDeEQsMEJBQTBCLG1CQUFPLENBQUMsdUZBQXVDO0FBQ3pFLDBCQUEwQixtQkFBTyxDQUFDLHVGQUF1QztBQUN6RSxhQUFhLG1CQUFPLENBQUMsbURBQWtCO0FBQ3ZDOzs7Ozs7Ozs7OztBQ2RBLGVBQWUsbUJBQU8sQ0FBQyxpRUFBOEI7QUFDckQsa0JBQWtCLG1CQUFPLENBQUMscUVBQWdDO0FBQzFELFlBQVksbUJBQU8sQ0FBQyx5REFBMEI7QUFDOUMsZUFBZSxtQkFBTyxDQUFDLCtEQUE2QjtBQUNwRCxZ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R0FBRzs7QUFFSDtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0EsR0FBRzs7QUFFSDtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQSxpQkFBaUIsUUFBUTtBQUN6QjtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0EsbUJBQW1CLFFBQVE7QUFDM0I7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0EsR0FBRzs7QUFFSDtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQSxlQUFlLE9BQU87QUFDdEI7QUFDQTtBQUNBO0FBQ0EsR0FBRztBQUNIOztBQUVBO0FBQ0E7QUFDQTtBQUNBO0FBQ0EsR0FBRztBQUNIO0FBQ0E7QUFDQTs7Ozs7OztVQ3hNQTtVQUNBOztVQUVBO1VBQ0E7VUFDQTtVQUNBO1VBQ0E7VUFDQTtVQUNBO1VBQ0E7VUFDQTtVQUNBO1VBQ0E7VUFDQTs7VUFFQTtVQUNBOztVQUVBO1VBQ0E7VUFDQTs7O1VDckJBO1VBQ0E7VUFDQTtVQUNBIiwiZmlsZSI6ImthbG1hbi1maWx0ZXIuanMiLCJzb3VyY2VzQ29udGVudCI6WyJjb25zdCBtb2RlbENvbGxlY3Rpb24gPSByZXF1aXJlKCcuL2xpYi9tb2RlbC1jb2xsZWN0aW9uJyk7XG5cbm1vZHVsZS5leHBvcnRzID0ge1xuXHRyZWdpc3RlckR5bmFtaWM6IG1vZGVsQ29sbGVjdGlvbi5yZWdpc3RlckR5bmFtaWMsXG5cdEthbG1hbkZpbHRlcjogcmVxdWlyZSgnLi9saWIva2FsbWFuLWZpbHRlcicpLFxuXHRyZWdpc3Rlck9ic2VydmF0aW9uOiBtb2RlbENvbGxlY3Rpb24ucmVnaXN0ZXJPYnNlcnZhdGlvbixcblx0YnVpbGRPYnNlcnZhdGlvbjogbW9kZWxDb2xsZWN0aW9uLmJ1aWxkT2JzZXJ2YXRpb24sXG5cdGJ1aWxkRHluYW1pYzogbW9kZWxDb2xsZWN0aW9uLmJ1aWxkRHluYW1pYyxcblx0Z2V0Q292YXJpYW5jZTogcmVxdWlyZSgnLi9saWIvdXRpbHMvZ2V0LWNvdmFyaWFuY2UnKSxcblx0U3RhdGU6IHJlcXVpcmUoJy4vbGliL3N0YXRlJyksXG5cdGNoZWNrQ292YXJpYW5jZTogcmVxdWlyZSgnLi9saWIvdXRpbHMvY2hlY2stY292YXJpYW5jZScpLFxuXHRjb3JyZWxhdGlvblRvQ292YXJpYW5jZTogcmVxdWlyZSgnLi9saWIvdXRpbHMvY29ycmVsYXRpb24tdG8tY292YXJpYW5jZScpLFxuXHRjb3ZhcmlhbmNlVG9Db3JyZWxhdGlvbjogcmVxdWlyZSgnLi9saWIvdXRpbHMvY292YXJpYW5jZS10by1jb3JyZWxhdGlvbicpLFxuXHRsaW5hbGdlYnJhOiByZXF1aXJlKCcuL2xpYi9saW5hbGdlYnJhJylcbn07XG4iLCJjb25zdCBtYXRNdWwgPSByZXF1aXJlKCcuLi9saWIvbGluYWxnZWJyYS9tYXQtbXVsLmpzJyk7XG5jb25zdCB0cmFuc3Bvc2UgPSByZXF1aXJlKCcuLi9saWIvbGluYWxnZWJyYS90cmFuc3Bvc2UuanMnKTtcbmNvbnN0IGFkZCA9IHJlcXVpcmUoJy4uL2xpYi9saW5hbGdlYnJhL2FkZC5qcycpO1xuY29uc3QgaW52ZXJ0ID0gcmVxdWlyZSgnLi4vbGliL2xpbmFsZ2VicmEvaW52ZXJ0LmpzJyk7XG5jb25zdCBzdWIgPSByZXF1aXJlKCcuLi9saWIvbGluYWxnZWJyYS9zdWIuanMnKTtcbmNvbnN0IGdldElkZW50aXR5ID0gcmVxdWlyZSgnLi4vbGliL2xpbmFsZ2VicmEvaWRlbnRpdHkuanMnKTtcbmNvbnN0IFN0YXRlID0gcmVxdWlyZSgnLi9zdGF0ZS5qcycpO1xuY29uc3QgY2hlY2tNYXRyaXggPSByZXF1aXJlKCcuL3V0aWxzL2NoZWNrLW1hdHJpeC5qcycpO1xuLyoqXG4qIEBjYWxsYmFjayBPYnNlcnZhdGlvbkNhbGxiYWNrXG4qIEBwYXJhbSB7T2JqZWN0fSBvcHRzXG4qIEBwYXJhbSB7TnVtYmVyfSBvcHRzLmluZGV4XG4qIEBwYXJhbSB7TnVtYmVyfSBvcHRzLnByZXZpb3VzQ29ycmVjdGVkXG4qL1xuXG4vKipcbiogQHR5cGVkZWYge09iamVjdH0gT2JzZXJ2YXRpb25Db25maWdcbiogQHByb3BlcnR5IHtOdW1iZXJ9IGRpbWVuc2lvblxuKiBAcHJvcGVydHkge0FycmF5LkFycmF5LjxOdW1iZXI+PiB8IE9ic2VydmF0aW9uQ2FsbGJhY2t9IHN0YXRlUHJvamVjdGlvbixcbiogQHByb3BlcnR5IHtBcnJheS5BcnJheS48TnVtYmVyPj4gfCBPYnNlcnZhdGlvbkNhbGxiYWNrfSBjb3ZhcmlhbmNlXG4qL1xuXG4vKipcbiogQGNhbGxiYWNrIER5bmFtaWNDYWxsYmFja1xuKiBAcGFyYW0ge09iamVjdH0gb3B0c1xuKiBAcGFyYW0ge051bWJlcn0gb3B0cy5pbmRleFxuKiBAcGFyYW0ge1N0YXRlfSBvcHRzLnByZWRpY3RlZFxuKiBAcGFyYW0ge09ic2VydmF0aW9ufSBvcHRzLm9ic2VydmF0aW9uXG4qL1xuXG4vKipcbiogQHR5cGVkZWYge09iamVjdH0gRHluYW1pY0NvbmZpZ1xuKiBAcHJvcGVydHkge051bWJlcn0gZGltZW5zaW9uXG4qIEBwcm9wZXJ0eSB7QXJyYXkuQXJyYXkuPE51bWJlcj4+IHwgRHluYW1pY0NhbGxiYWNrfSB0cmFuc2l0aW9uLFxuKiBAcHJvcGVydHkge0FycmF5LkFycmF5LjxOdW1iZXI+PiB8IER5bmFtaWNDYWxsYmFja30gY292YXJpYW5jZVxuKi9cblxuY29uc3QgZGVmYXVsdExvZ2dlciA9IHtcblx0aW5mbzogKC4uLmFyZ3MpID0+IGNvbnNvbGUubG9nKC4uLmFyZ3MpLFxuXHRkZWJ1ZzogKCkgPT4ge30sXG5cdHdhcm46ICguLi5hcmdzKSA9PiBjb25zb2xlLmxvZyguLi5hcmdzKSxcblx0ZXJyb3I6ICguLi5hcmdzKSA9PiBjb25zb2xlLmxvZyguLi5hcmdzKVxufTtcblxuLyoqXG4qIEBjbGFzc1xuKiBAcHJvcGVydHkge0R5bmFtaWNDb25maWd9IGR5bmFtaWMgdGhlIHN5c3RlbSdzIGR5bmFtaWMgbW9kZWxcbiogQHByb3BlcnR5IHtPYnNlcnZhdGlvbkNvbmZpZ30gb2JzZXJ2YXRpb24gdGhlIHN5c3RlbSdzIG9ic2VydmF0aW9uIG1vZGVsXG4qQHByb3BlcnR5IGxvZ2dlciBhIFdpbnN0b24tbGlrZSBsb2dnZXJcbiovXG5jbGFzcyBDb3JlS2FsbWFuRmlsdGVyIHtcblx0LyoqXG5cdCogQHBhcmFtIHtEeW5hbWljQ29uZmlnfSBkeW5hbWljXG5cdCogQHBhcmFtIHtPYnNlcnZhdGlvbkNvbmZpZ30gb2JzZXJ2YXRpb24gdGhlIHN5c3RlbSdzIG9ic2VydmF0aW9uIG1vZGVsXG5cdCovXG5cblx0Y29uc3RydWN0b3Ioe2R5bmFtaWMsIG9ic2VydmF0aW9uLCBsb2dnZXIgPSBkZWZhdWx0TG9nZ2VyfSkge1xuXHRcdHRoaXMuZHluYW1pYyA9IGR5bmFtaWM7XG5cdFx0dGhpcy5vYnNlcnZhdGlvbiA9IG9ic2VydmF0aW9uO1xuXHRcdHRoaXMubG9nZ2VyID0gbG9nZ2VyO1xuXHR9XG5cblx0Z2V0VmFsdWUoZm4sIG9wdGlvbnMpIHtcblx0XHRyZXR1cm4gKHR5cGVvZiAoZm4pID09PSAnZnVuY3Rpb24nID8gZm4ob3B0aW9ucykgOiBmbik7XG5cdH1cblxuXHRnZXRJbml0U3RhdGUoKSB7XG5cdFx0Y29uc3Qge21lYW46IG1lYW5Jbml0LCBjb3ZhcmlhbmNlOiBjb3ZhcmlhbmNlSW5pdCwgaW5kZXg6IGluZGV4SW5pdH0gPSB0aGlzLmR5bmFtaWMuaW5pdDtcblx0XHRjb25zdCBpbml0U3RhdGUgPSBuZXcgU3RhdGUoe1xuXHRcdFx0bWVhbjogbWVhbkluaXQsXG5cdFx0XHRjb3ZhcmlhbmNlOiBjb3ZhcmlhbmNlSW5pdCxcblx0XHRcdGluZGV4OiBpbmRleEluaXRcblx0XHR9KTtcblx0XHRyZXR1cm4gaW5pdFN0YXRlO1xuXHR9XG5cblx0LyoqXG5cdFRoaXMgd2lsbCByZXR1cm4gdGhlIHByZWRpY3RlZCBjb3ZhcmlhbmNlIG9mIGEgZ2l2ZW4gcHJldmlvdXNDb3JyZWN0ZWQgU3RhdGUsIHRoaXMgd2lsbCBoZWxwIHVzIHRvIGJ1aWxkIHRoZSBhc3ltcHRvdGljU3RhdGUuXG5cdCogQHBhcmFtIHtTdGF0ZX0gcHJldmlvdXNDb3JyZWN0ZWRcblx0KiBAcmV0dXJuc3tBcnJheS48QXJyYXkuPE51bWJlcj4+fVxuXHQqL1xuXG5cdGdldFByZWRpY3RlZENvdmFyaWFuY2Uob3B0aW9ucyA9IHt9KSB7XG5cdFx0bGV0IHtwcmV2aW91c0NvcnJlY3RlZCwgaW5kZXh9ID0gb3B0aW9ucztcblx0XHRwcmV2aW91c0NvcnJlY3RlZCA9IHByZXZpb3VzQ29ycmVjdGVkIHx8IHRoaXMuZ2V0SW5pdFN0YXRlKCk7XG5cblx0XHRjb25zdCBnZXRWYWx1ZU9wdGlvbnMgPSBPYmplY3QuYXNzaWduKHt9LCB7cHJldmlvdXNDb3JyZWN0ZWQsIGluZGV4fSwgb3B0aW9ucyk7XG5cdFx0Y29uc3QgZCA9IHRoaXMuZ2V0VmFsdWUodGhpcy5keW5hbWljLnRyYW5zaXRpb24sIGdldFZhbHVlT3B0aW9ucyk7XG5cdFx0Y29uc3QgZFRyYW5zcG9zZWQgPSB0cmFuc3Bvc2UoZCk7XG5cdFx0Y29uc3QgY292YXJpYW5jZUludGVyID0gbWF0TXVsKGQsIHByZXZpb3VzQ29ycmVjdGVkLmNvdmFyaWFuY2UpO1xuXHRcdGNvbnN0IGNvdmFyaWFuY2VQcmV2aW91cyA9IG1hdE11bChjb3ZhcmlhbmNlSW50ZXIsIGRUcmFuc3Bvc2VkKTtcblx0XHRjb25zdCBkeW5Db3YgPSB0aGlzLmdldFZhbHVlKHRoaXMuZHluYW1pYy5jb3ZhcmlhbmNlLCBnZXRWYWx1ZU9wdGlvbnMpO1xuXG5cdFx0Y29uc3QgY292YXJpYW5jZSA9IGFkZChcblx0XHRcdGR5bkNvdixcblx0XHRcdGNvdmFyaWFuY2VQcmV2aW91c1xuXHRcdCk7XG5cdFx0Y2hlY2tNYXRyaXgoY292YXJpYW5jZSwgW3RoaXMuZHluYW1pYy5kaW1lbnNpb24sIHRoaXMuZHluYW1pYy5kaW1lbnNpb25dLCAncHJlZGljdGVkLmNvdmFyaWFuY2UnKTtcblxuXHRcdHJldHVybiBjb3ZhcmlhbmNlO1xuXHR9XG5cblx0LyoqXG5cdFRoaXMgd2lsbCByZXR1cm4gdGhlIG5ldyBwcmVkaWN0aW9uLCByZWxhdGl2ZWx5IHRvIHRoZSBkeW5hbWljIG1vZGVsIGNob3NlblxuXHQqIEBwYXJhbSB7U3RhdGV9IHByZXZpb3VzQ29ycmVjdGVkIFN0YXRlIHJlbGF0aXZlIHRvIG91ciBkeW5hbWljIG1vZGVsXG5cdCogQHJldHVybnN7U3RhdGV9IHByZWRpY3RlZCBTdGF0ZVxuXHQqL1xuXG5cdHByZWRpY3Qob3B0aW9ucyA9IHt9KSB7XG5cdFx0bGV0IHtwcmV2aW91c0NvcnJlY3RlZCwgaW5kZXh9ID0gb3B0aW9ucztcblx0XHRwcmV2aW91c0NvcnJlY3RlZCA9IHByZXZpb3VzQ29ycmVjdGVkIHx8IHRoaXMuZ2V0SW5pdFN0YXRlKCk7XG5cblx0XHRpZiAodHlwZW9mIChpbmRleCkgIT09ICdudW1iZXInICYmIHR5cGVvZiAocHJldmlvdXNDb3JyZWN0ZWQuaW5kZXgpID09PSAnbnVtYmVyJykge1xuXHRcdFx0aW5kZXggPSBwcmV2aW91c0NvcnJlY3RlZC5pbmRleCArIDE7XG5cdFx0fVxuXG5cdFx0U3RhdGUuY2hlY2socHJldmlvdXNDb3JyZWN0ZWQsIHtkaW1lbnNpb246IHRoaXMuZHluYW1pYy5kaW1lbnNpb259KTtcblxuXHRcdGNvbnN0IGdldFZhbHVlT3B0aW9ucyA9IE9iamVjdC5hc3NpZ24oe30sIHtcblx0XHRcdHByZXZpb3VzQ29ycmVjdGVkLFxuXHRcdFx0aW5kZXhcblx0XHR9LCBvcHRpb25zKTtcblx0XHRjb25zdCBkID0gdGhpcy5nZXRWYWx1ZSh0aGlzLmR5bmFtaWMudHJhbnNpdGlvbiwgZ2V0VmFsdWVPcHRpb25zKTtcblxuXHRcdGNoZWNrTWF0cml4KGQsIFt0aGlzLmR5bmFtaWMuZGltZW5zaW9uLCB0aGlzLmR5bmFtaWMuZGltZW5zaW9uXSwgJ2R5bmFtaWMudHJhbnNpdGlvbicpO1xuXG5cdFx0Y29uc3QgbWVhbiA9IG1hdE11bChkLCBwcmV2aW91c0NvcnJlY3RlZC5tZWFuKTtcblxuXHRcdGNvbnN0IGNvdmFyaWFuY2UgPSB0aGlzLmdldFByZWRpY3RlZENvdmFyaWFuY2UoZ2V0VmFsdWVPcHRpb25zKTtcblxuXHRcdGNvbnN0IHByZWRpY3RlZCA9IG5ldyBTdGF0ZSh7bWVhbiwgY292YXJpYW5jZSwgaW5kZXh9KTtcblx0XHR0aGlzLmxvZ2dlci5kZWJ1ZygnUHJlZGljdGlvbiBkb25lJywgcHJlZGljdGVkKTtcblxuXHRcdHJldHVybiBwcmVkaWN0ZWQ7XG5cdH1cblx0LyoqXG5cdFRoaXMgd2lsbCByZXR1cm4gdGhlIG5ldyBjb3JyZWN0aW9uLCB0YWtpbmcgaW50byBhY2NvdW50IHRoZSBwcmVkaWN0aW9uIG1hZGVcblx0YW5kIHRoZSBvYnNlcnZhdGlvbiBvZiB0aGUgc2Vuc29yXG5cdCogQHBhcmFtIHtTdGF0ZX0gcHJlZGljdGVkIHRoZSBwcmV2aW91cyBTdGF0ZVxuXHQqIEByZXR1cm5ze0FycmF5PEFycmF5Pn0ga2FsbWFuR2FpblxuXHQqL1xuXG5cdGdldEdhaW4ob3B0aW9ucykge1xuXHRcdGxldCB7cHJlZGljdGVkLCBzdGF0ZVByb2plY3Rpb259ID0gb3B0aW9ucztcblx0XHRjb25zdCBnZXRWYWx1ZU9wdGlvbnMgPSBPYmplY3QuYXNzaWduKHt9LCB7aW5kZXg6IHByZWRpY3RlZC5pbmRleH0sIG9wdGlvbnMpO1xuXHRcdHN0YXRlUHJvamVjdGlvbiA9IHN0YXRlUHJvamVjdGlvbiB8fCB0aGlzLmdldFZhbHVlKHRoaXMub2JzZXJ2YXRpb24uc3RhdGVQcm9qZWN0aW9uLCBnZXRWYWx1ZU9wdGlvbnMpO1xuXHRcdGNvbnN0IG9ic0NvdmFyaWFuY2UgPSB0aGlzLmdldFZhbHVlKHRoaXMub2JzZXJ2YXRpb24uY292YXJpYW5jZSwgZ2V0VmFsdWVPcHRpb25zKTtcblx0XHRjaGVja01hdHJpeChvYnNDb3ZhcmlhbmNlLCBbdGhpcy5vYnNlcnZhdGlvbi5kaW1lbnNpb24sIHRoaXMub2JzZXJ2YXRpb24uZGltZW5zaW9uXSwgJ29ic2VydmF0aW9uLmNvdmFyaWFuY2UnKTtcblxuXHRcdGNvbnN0IHN0YXRlUHJvalRyYW5zcG9zZWQgPSB0cmFuc3Bvc2Uoc3RhdGVQcm9qZWN0aW9uKTtcblx0XHRjb25zdCBub2lzZWxlc3NJbm5vdmF0aW9uID0gbWF0TXVsKFxuXHRcdFx0bWF0TXVsKHN0YXRlUHJvamVjdGlvbiwgcHJlZGljdGVkLmNvdmFyaWFuY2UpLFxuXHRcdFx0c3RhdGVQcm9qVHJhbnNwb3NlZFxuXHRcdCk7XG5cblx0XHRjb25zdCBpbm5vdmF0aW9uQ292YXJpYW5jZSA9IGFkZChub2lzZWxlc3NJbm5vdmF0aW9uLCBvYnNDb3ZhcmlhbmNlKTtcblxuXHRcdGNvbnN0IG9wdGltYWxLYWxtYW5HYWluID0gbWF0TXVsKFxuXHRcdFx0bWF0TXVsKHByZWRpY3RlZC5jb3ZhcmlhbmNlLCBzdGF0ZVByb2pUcmFuc3Bvc2VkKSxcblx0XHRcdGludmVydChpbm5vdmF0aW9uQ292YXJpYW5jZSlcblx0XHQpO1xuXG5cdFx0cmV0dXJuIG9wdGltYWxLYWxtYW5HYWluO1xuXHR9XG5cblx0LyoqXG5cdFRoaXMgd2lsbCByZXR1cm4gdGhlIGNvcnJlY3RlZCBjb3ZhcmlhbmNlIG9mIGEgZ2l2ZW4gcHJlZGljdGVkIFN0YXRlLCB0aGlzIHdpbGwgaGVscCB1cyB0byBidWlsZCB0aGUgYXN5bXB0b3RpY1N0YXRlLlxuXHQqIEBwYXJhbSB7U3RhdGV9IHByZWRpY3RlZCB0aGUgcHJldmlvdXMgU3RhdGVcblx0KiBAcmV0dXJuc3tBcnJheS48QXJyYXkuPE51bWJlcj4+fVxuXHQqL1xuXG5cdGdldENvcnJlY3RlZENvdmFyaWFuY2Uob3B0aW9ucykge1xuXHRcdGxldCB7cHJlZGljdGVkLCBvcHRpbWFsS2FsbWFuR2Fpbiwgc3RhdGVQcm9qZWN0aW9ufSA9IG9wdGlvbnM7XG5cdFx0Y29uc3QgaWRlbnRpdHkgPSBnZXRJZGVudGl0eShwcmVkaWN0ZWQuY292YXJpYW5jZS5sZW5ndGgpO1xuXHRcdGlmICghc3RhdGVQcm9qZWN0aW9uKSB7XG5cdFx0XHRjb25zdCBnZXRWYWx1ZU9wdGlvbnMgPSBPYmplY3QuYXNzaWduKHt9LCB7aW5kZXg6IHByZWRpY3RlZC5pbmRleH0sIG9wdGlvbnMpO1xuXHRcdFx0c3RhdGVQcm9qZWN0aW9uID0gdGhpcy5nZXRWYWx1ZSh0aGlzLm9ic2VydmF0aW9uLnN0YXRlUHJvamVjdGlvbiwgZ2V0VmFsdWVPcHRpb25zKTtcblx0XHR9XG5cblx0XHRpZiAoIW9wdGltYWxLYWxtYW5HYWluKSB7XG5cdFx0XHRvcHRpbWFsS2FsbWFuR2FpbiA9IHRoaXMuZ2V0R2FpbihPYmplY3QuYXNzaWduKHtzdGF0ZVByb2plY3Rpb259LCBvcHRpb25zKSk7XG5cdFx0fVxuXG5cdFx0cmV0dXJuIG1hdE11bChcblx0XHRcdHN1YihpZGVudGl0eSwgbWF0TXVsKG9wdGltYWxLYWxtYW5HYWluLCBzdGF0ZVByb2plY3Rpb24pKSxcblx0XHRcdHByZWRpY3RlZC5jb3ZhcmlhbmNlXG5cdFx0KTtcblx0fVxuXG5cdC8qKlxuXHRUaGlzIHdpbGwgcmV0dXJuIHRoZSBuZXcgY29ycmVjdGlvbiwgdGFraW5nIGludG8gYWNjb3VudCB0aGUgcHJlZGljdGlvbiBtYWRlXG5cdGFuZCB0aGUgb2JzZXJ2YXRpb24gb2YgdGhlIHNlbnNvclxuXHQqIEBwYXJhbSB7U3RhdGV9IHByZWRpY3RlZCB0aGUgcHJldmlvdXMgU3RhdGVcblx0KiBAcGFyYW0ge0FycmF5fSBvYnNlcnZhdGlvbiB0aGUgb2JzZXJ2YXRpb24gb2YgdGhlIHNlbnNvclxuXHQqIEByZXR1cm5ze1N0YXRlfSBjb3JyZWN0ZWQgU3RhdGUgb2YgdGhlIEthbG1hbiBGaWx0ZXJcblx0Ki9cblxuXHRjb3JyZWN0KG9wdGlvbnMpIHtcblx0XHRjb25zdCB7cHJlZGljdGVkLCBvYnNlcnZhdGlvbn0gPSBvcHRpb25zO1xuXHRcdFN0YXRlLmNoZWNrKHByZWRpY3RlZCwge2RpbWVuc2lvbjogdGhpcy5keW5hbWljLmRpbWVuc2lvbn0pO1xuXHRcdGlmICghb2JzZXJ2YXRpb24pIHtcblx0XHRcdHRocm93IChuZXcgRXJyb3IoJ25vIG1lYXN1cmUgYXZhaWxhYmxlJykpO1xuXHRcdH1cblxuXHRcdGNvbnN0IGdldFZhbHVlT3B0aW9ucyA9IE9iamVjdC5hc3NpZ24oe30sIHtvYnNlcnZhdGlvbiwgcHJlZGljdGVkLCBpbmRleDogcHJlZGljdGVkLmluZGV4fSwgb3B0aW9ucyk7XG5cdFx0Y29uc3Qgc3RhdGVQcm9qZWN0aW9uID0gdGhpcy5nZXRWYWx1ZSh0aGlzLm9ic2VydmF0aW9uLnN0YXRlUHJvamVjdGlvbiwgZ2V0VmFsdWVPcHRpb25zKTtcblxuXHRcdGNvbnN0IG9wdGltYWxLYWxtYW5HYWluID0gdGhpcy5nZXRHYWluKE9iamVjdC5hc3NpZ24oe3ByZWRpY3RlZCwgc3RhdGVQcm9qZWN0aW9ufSwgb3B0aW9ucykpO1xuXG5cdFx0Y29uc3QgaW5ub3ZhdGlvbiA9IHN1Yihcblx0XHRcdG9ic2VydmF0aW9uLFxuXHRcdFx0bWF0TXVsKHN0YXRlUHJvamVjdGlvbiwgcHJlZGljdGVkLm1lYW4pXG5cdFx0KTtcblx0XHRjb25zdCBtZWFuID0gYWRkKFxuXHRcdFx0cHJlZGljdGVkLm1lYW4sXG5cdFx0XHRtYXRNdWwob3B0aW1hbEthbG1hbkdhaW4sIGlubm92YXRpb24pXG5cdFx0KTtcblx0XHRpZiAoTnVtYmVyLmlzTmFOKG1lYW5bMF1bMF0pKSB7XG5cdFx0XHRjb25zb2xlLmxvZyh7b3B0aW1hbEthbG1hbkdhaW4sIGlubm92YXRpb24sIHByZWRpY3RlZH0pO1xuXHRcdFx0dGhyb3cgKG5ldyBUeXBlRXJyb3IoJ01lYW4gaXMgTmFOIGFmdGVyIGNvcnJlY3Rpb24nKSk7XG5cdFx0fVxuXG5cdFx0Y29uc3QgY292YXJpYW5jZSA9IHRoaXMuZ2V0Q29ycmVjdGVkQ292YXJpYW5jZShPYmplY3QuYXNzaWduKHtwcmVkaWN0ZWQsIG9wdGltYWxLYWxtYW5HYWluLCBzdGF0ZVByb2plY3Rpb259LCBvcHRpb25zKSk7XG5cdFx0Y29uc3QgY29ycmVjdGVkID0gbmV3IFN0YXRlKHttZWFuLCBjb3ZhcmlhbmNlLCBpbmRleDogcHJlZGljdGVkLmluZGV4fSk7XG5cdFx0dGhpcy5sb2dnZXIuZGVidWcoJ0NvcnJlY3Rpb24gZG9uZScsIGNvcnJlY3RlZCk7XG5cdFx0cmV0dXJuIGNvcnJlY3RlZDtcblx0fVxufVxuXG5tb2R1bGUuZXhwb3J0cyA9IENvcmVLYWxtYW5GaWx0ZXI7XG4iLCJjb25zdCBpZGVudGl0eSA9IHJlcXVpcmUoJy4uL2xpbmFsZ2VicmEvaWRlbnRpdHkuanMnKTtcblxuLyoqXG4qQ3JlYXRlcyBhIGR5bmFtaWMgbW9kZWwsIGZvbGxvd2luZyBjb25zdGFudCBhY2NlbGVyYXRpb24gbW9kZWwgd2l0aCByZXNwZWN0IHdpdGggdGhlIGRpbWVuc2lvbnMgcHJvdmlkZWQgaW4gdGhlIG9ic2VydmF0aW9uIHBhcmFtZXRlcnNcbiogQHBhcmFtIHtEeW5hbWljQ29uZmlnfSBkeW5hbWljXG4qIEBwYXJhbSB7T2JzZXJ2YXRpb25Db25maWd9IG9ic2VydmF0aW9uXG4qIEByZXR1cm5zIHtEeW5hbWljQ29uZmlnfVxuKi9cblxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiAoZHluYW1pYywgb2JzZXJ2YXRpb24pIHtcblx0Y29uc3QgdGltZVN0ZXAgPSBkeW5hbWljLnRpbWVTdGVwIHx8IDE7XG5cdGNvbnN0IHtvYnNlcnZlZFByb2plY3Rpb259ID0gb2JzZXJ2YXRpb247XG5cdGNvbnN0IHtzdGF0ZVByb2plY3Rpb259ID0gb2JzZXJ2YXRpb247XG5cdGNvbnN0IG9ic2VydmF0aW9uRGltZW5zaW9uID0gb2JzZXJ2YXRpb24uZGltZW5zaW9uO1xuXHRsZXQgZGltZW5zaW9uO1xuXG5cdGlmIChzdGF0ZVByb2plY3Rpb24gJiYgTnVtYmVyLmlzSW50ZWdlcihzdGF0ZVByb2plY3Rpb25bMF0ubGVuZ3RoIC8gMykpIHtcblx0XHRkaW1lbnNpb24gPSBvYnNlcnZhdGlvbi5zdGF0ZVByb2plY3Rpb25bMF0ubGVuZ3RoO1xuXHR9IGVsc2UgaWYgKG9ic2VydmVkUHJvamVjdGlvbikge1xuXHRcdGRpbWVuc2lvbiA9IG9ic2VydmVkUHJvamVjdGlvblswXS5sZW5ndGggKiAzO1xuXHR9IGVsc2UgaWYgKG9ic2VydmF0aW9uRGltZW5zaW9uKSB7XG5cdFx0ZGltZW5zaW9uID0gb2JzZXJ2YXRpb25EaW1lbnNpb24gKiAzO1xuXHR9IGVsc2Uge1xuXHRcdHRocm93IChuZXcgRXJyb3IoJ29ic2VydmVkUHJvamVjdGlvbiBvciBzdGF0ZVByb2plY3Rpb24gc2hvdWxkIGJlIGRlZmluZWQgaW4gb2JzZXJ2YXRpb24gaW4gb3JkZXIgdG8gdXNlIGNvbnN0YW50LXNwZWVkIGZpbHRlcicpKTtcblx0fVxuXG5cdGNvbnN0IGJhc2VEaW1lbnNpb24gPSBkaW1lbnNpb24gLyAzO1xuXHQvLyBXZSBjb25zdHJ1Y3QgdGhlIHRyYW5zaXRpb24gYW5kIGNvdmFyaWFuY2UgbWF0cmljZXNcblx0Y29uc3QgdHJhbnNpdGlvbiA9IGlkZW50aXR5KGRpbWVuc2lvbik7XG5cdGZvciAobGV0IGkgPSAwOyBpIDwgYmFzZURpbWVuc2lvbjsgaSsrKSB7XG5cdFx0dHJhbnNpdGlvbltpXVtpICsgYmFzZURpbWVuc2lvbl0gPSB0aW1lU3RlcDtcblx0XHR0cmFuc2l0aW9uW2ldW2kgKyAoMiAqIGJhc2VEaW1lbnNpb24pXSA9IDAuNSAqICh0aW1lU3RlcCAqKiAyKTtcblx0XHR0cmFuc2l0aW9uW2kgKyBiYXNlRGltZW5zaW9uXVtpICsgKDIgKiBiYXNlRGltZW5zaW9uKV0gPSB0aW1lU3RlcDtcblx0fVxuXG5cdGNvbnN0IGFycmF5Q292YXJpYW5jZSA9IG5ldyBBcnJheShiYXNlRGltZW5zaW9uKS5maWxsKDEpXG5cdFx0LmNvbmNhdChuZXcgQXJyYXkoYmFzZURpbWVuc2lvbikuZmlsbCh0aW1lU3RlcCAqIHRpbWVTdGVwKSlcblx0XHQuY29uY2F0KG5ldyBBcnJheShiYXNlRGltZW5zaW9uKS5maWxsKHRpbWVTdGVwICoqIDQpKTtcblx0Y29uc3QgY292YXJpYW5jZSA9IGR5bmFtaWMuY292YXJpYW5jZSB8fCBhcnJheUNvdmFyaWFuY2U7XG5cdHJldHVybiBPYmplY3QuYXNzaWduKHt9LCBkeW5hbWljLCB7ZGltZW5zaW9uLCB0cmFuc2l0aW9uLCBjb3ZhcmlhbmNlfSk7XG59O1xuIiwiY29uc3QgaWRlbnRpdHkgPSByZXF1aXJlKCcuLi9saW5hbGdlYnJhL2lkZW50aXR5LmpzJyk7XG4vKipcbipDcmVhdGVzIGEgZHluYW1pYyBtb2RlbCwgZm9sbG93aW5nIGNvbnN0YW50IHBvc2l0aW9uIG1vZGVsIHdpdGggcmVzcGVjdCB3aXRoIHRoZSBkaW1lbnNpb25zIHByb3ZpZGVkIGluIHRoZSBvYnNlcnZhdGlvbiBwYXJhbWV0ZXJzXG4qIEBwYXJhbSB7RHluYW1pY0NvbmZpZ30gZHluYW1pY1xuKiBAcGFyYW0ge09ic2VydmF0aW9uQ29uZmlnfSBvYnNlcnZhdGlvblxuKiBAcmV0dXJucyB7RHluYW1pY0NvbmZpZ31cbiovXG5cbm1vZHVsZS5leHBvcnRzID0gZnVuY3Rpb24gKGR5bmFtaWMsIG9ic2VydmF0aW9uKSB7XG5cdGxldCB7ZGltZW5zaW9ufSA9IGR5bmFtaWM7XG5cdGNvbnN0IG9ic2VydmF0aW9uRGltZW5zaW9uID0gb2JzZXJ2YXRpb24uZGltZW5zaW9uO1xuXHRjb25zdCB7b2JzZXJ2ZWRQcm9qZWN0aW9ufSA9IG9ic2VydmF0aW9uO1xuXHRjb25zdCB7c3RhdGVQcm9qZWN0aW9ufSA9IG9ic2VydmF0aW9uO1xuXHRsZXQge2NvdmFyaWFuY2V9ID0gZHluYW1pYztcblxuXHRpZiAoIWR5bmFtaWMuZGltZW5zaW9uKSB7XG5cdFx0aWYgKG9ic2VydmF0aW9uRGltZW5zaW9uKSB7XG5cdFx0XHRkaW1lbnNpb24gPSBvYnNlcnZhdGlvbkRpbWVuc2lvbjtcblx0XHR9IGVsc2UgaWYgKG9ic2VydmVkUHJvamVjdGlvbikge1xuXHRcdFx0ZGltZW5zaW9uID0gb2JzZXJ2ZWRQcm9qZWN0aW9uWzBdLmxlbmd0aDtcblx0XHR9IGVsc2UgaWYgKHN0YXRlUHJvamVjdGlvbikge1xuXHRcdFx0ZGltZW5zaW9uID0gc3RhdGVQcm9qZWN0aW9uWzBdLmxlbmd0aDtcblx0XHR9XG5cdH1cblxuXHRjb25zdCB0cmFuc2l0aW9uID0gaWRlbnRpdHkoZGltZW5zaW9uKTtcblx0Y292YXJpYW5jZSA9IGNvdmFyaWFuY2UgfHwgaWRlbnRpdHkoZGltZW5zaW9uKTtcblx0cmV0dXJuIE9iamVjdC5hc3NpZ24oe30sIGR5bmFtaWMsIHtkaW1lbnNpb24sIHRyYW5zaXRpb24sIGNvdmFyaWFuY2V9KTtcbn07XG4iLCJjb25zdCBpZGVudGl0eSA9IHJlcXVpcmUoJy4uL2xpbmFsZ2VicmEvaWRlbnRpdHkuanMnKTtcblxuLyoqXG4qQ3JlYXRlcyBhIGR5bmFtaWMgbW9kZWwsIGZvbGxvd2luZyBjb25zdGFudCBwb3NpdGlvbiBtb2RlbCB3aXRoIHJlc3BlY3Qgd2l0aCB0aGUgZGltZW5zaW9ucyBwcm92aWRlZCBpbiB0aGUgb2JzZXJ2YXRpb24gcGFyYW1ldGVyc1xuKiBAcGFyYW0ge0R5bmFtaWNDb25maWd9IGR5bmFtaWNcbiogQHBhcmFtIHtPYnNlcnZhdGlvbkNvbmZpZ30gb2JzZXJ2YXRpb25cbiogQHJldHVybnMge0R5bmFtaWNDb25maWd9XG4qL1xuXG5tb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uIChkeW5hbWljLCBvYnNlcnZhdGlvbikge1xuXHRjb25zdCB0aW1lU3RlcCA9IGR5bmFtaWMudGltZVN0ZXAgfHwgMTtcblx0Y29uc3Qge29ic2VydmVkUHJvamVjdGlvbn0gPSBvYnNlcnZhdGlvbjtcblx0Y29uc3Qge3N0YXRlUHJvamVjdGlvbn0gPSBvYnNlcnZhdGlvbjtcblx0Y29uc3Qgb2JzZXJ2YXRpb25EaW1lbnNpb24gPSBvYnNlcnZhdGlvbi5kaW1lbnNpb247XG5cdGxldCBkaW1lbnNpb247XG5cblx0aWYgKHN0YXRlUHJvamVjdGlvbiAmJiBOdW1iZXIuaXNJbnRlZ2VyKHN0YXRlUHJvamVjdGlvblswXS5sZW5ndGggLyAyKSkge1xuXHRcdGRpbWVuc2lvbiA9IG9ic2VydmF0aW9uLnN0YXRlUHJvamVjdGlvblswXS5sZW5ndGg7XG5cdH0gZWxzZSBpZiAob2JzZXJ2ZWRQcm9qZWN0aW9uKSB7XG5cdFx0ZGltZW5zaW9uID0gb2JzZXJ2ZWRQcm9qZWN0aW9uWzBdLmxlbmd0aCAqIDI7XG5cdH0gZWxzZSBpZiAob2JzZXJ2YXRpb25EaW1lbnNpb24pIHtcblx0XHRkaW1lbnNpb24gPSBvYnNlcnZhdGlvbkRpbWVuc2lvbiAqIDI7XG5cdH0gZWxzZSB7XG5cdFx0dGhyb3cgKG5ldyBFcnJvcignb2JzZXJ2ZWRQcm9qZWN0aW9uIG9yIHN0YXRlUHJvamVjdGlvbiBzaG91bGQgYmUgZGVmaW5lZCBpbiBvYnNlcnZhdGlvbiBpbiBvcmRlciB0byB1c2UgY29uc3RhbnQtc3BlZWQgZmlsdGVyJykpO1xuXHR9XG5cblx0Y29uc3QgYmFzZURpbWVuc2lvbiA9IGRpbWVuc2lvbiAvIDI7XG5cdC8vIFdlIGNvbnN0cnVjdCB0aGUgdHJhbnNpdGlvbiBhbmQgY292YXJpYW5jZSBtYXRyaWNlc1xuXHRjb25zdCB0cmFuc2l0aW9uID0gaWRlbnRpdHkoZGltZW5zaW9uKTtcblx0Zm9yIChsZXQgaSA9IDA7IGkgPCBiYXNlRGltZW5zaW9uOyBpKyspIHtcblx0XHR0cmFuc2l0aW9uW2ldW2kgKyBiYXNlRGltZW5zaW9uXSA9IHRpbWVTdGVwO1xuXHR9XG5cblx0Y29uc3QgYXJyYXlDb3ZhcmlhbmNlID0gbmV3IEFycmF5KGJhc2VEaW1lbnNpb24pLmZpbGwoMSkuY29uY2F0KG5ldyBBcnJheShiYXNlRGltZW5zaW9uKS5maWxsKHRpbWVTdGVwICogdGltZVN0ZXApKTtcblx0Y29uc3QgY292YXJpYW5jZSA9IGR5bmFtaWMuY292YXJpYW5jZSB8fCBhcnJheUNvdmFyaWFuY2U7XG5cdHJldHVybiBPYmplY3QuYXNzaWduKHt9LCBkeW5hbWljLCB7ZGltZW5zaW9uLCB0cmFuc2l0aW9uLCBjb3ZhcmlhbmNlfSk7XG59O1xuIiwiXG5jb25zdCBhcnJheVRvTWF0cml4ID0gcmVxdWlyZSgnLi4vbGliL3V0aWxzL2FycmF5LXRvLW1hdHJpeC5qcycpO1xuY29uc3Qgc2V0RGltZW5zaW9ucyA9IHJlcXVpcmUoJy4uL2xpYi9zZXR1cC9zZXQtZGltZW5zaW9ucy5qcycpO1xuY29uc3QgY2hlY2tEaW1lbnNpb25zID0gcmVxdWlyZSgnLi4vbGliL3NldHVwL2NoZWNrLWRpbWVuc2lvbnMuanMnKTtcbmNvbnN0IGJ1aWxkU3RhdGVQcm9qZWN0aW9uID0gcmVxdWlyZSgnLi4vbGliL3NldHVwL2J1aWxkLXN0YXRlLXByb2plY3Rpb24uanMnKTtcbmNvbnN0IGV4dGVuZER5bmFtaWNJbml0ID0gcmVxdWlyZSgnLi4vbGliL3NldHVwL2V4dGVuZC1keW5hbWljLWluaXQuanMnKTtcbmNvbnN0IHRvRnVuY3Rpb24gPSByZXF1aXJlKCcuLi9saWIvdXRpbHMvdG8tZnVuY3Rpb24uanMnKTtcbmNvbnN0IGRlZXBBc3NpZ24gPSByZXF1aXJlKCcuLi9saWIvdXRpbHMvZGVlcC1hc3NpZ24uanMnKTtcbmNvbnN0IHBvbHltb3JwaE1hdHJpeCA9IHJlcXVpcmUoJy4uL2xpYi91dGlscy9wb2x5bW9ycGgtbWF0cml4LmpzJyk7XG5jb25zdCBkaXN0YW5jZU1hdCA9IHJlcXVpcmUoJy4uL2xpYi9saW5hbGdlYnJhL2Rpc3RhbmNlLW1hdC5qcycpO1xuY29uc3QgU3RhdGUgPSByZXF1aXJlKCcuL3N0YXRlLmpzJyk7XG5jb25zdCBtb2RlbENvbGxlY3Rpb24gPSByZXF1aXJlKCcuL21vZGVsLWNvbGxlY3Rpb24uanMnKTtcbmNvbnN0IENvcmVLYWxtYW5GaWx0ZXIgPSByZXF1aXJlKCcuL2NvcmUta2FsbWFuLWZpbHRlci5qcycpO1xuXG5jb25zdCBidWlsZERlZmF1bHREeW5hbWljID0gZnVuY3Rpb24gKGR5bmFtaWMpIHtcblx0aWYgKHR5cGVvZiAoZHluYW1pYykgPT09ICdzdHJpbmcnKSB7XG5cdFx0cmV0dXJuIHtuYW1lOiBkeW5hbWljfTtcblx0fVxuXG5cdHJldHVybiB7bmFtZTogJ2NvbnN0YW50LXBvc2l0aW9uJ307XG59O1xuXG5jb25zdCBidWlsZERlZmF1bHRPYnNlcnZhdGlvbiA9IGZ1bmN0aW9uIChvYnNlcnZhdGlvbikge1xuXHRpZiAodHlwZW9mIChvYnNlcnZhdGlvbikgPT09ICdudW1iZXInKSB7XG5cdFx0cmV0dXJuIHtuYW1lOiAnc2Vuc29yJywgc2Vuc29yRGltZW5zaW9uOiBvYnNlcnZhdGlvbn07XG5cdH1cblxuXHRpZiAodHlwZW9mIChvYnNlcnZhdGlvbikgPT09ICdzdHJpbmcnKSB7XG5cdFx0cmV0dXJuIHtuYW1lOiBvYnNlcnZhdGlvbn07XG5cdH1cblxuXHRyZXR1cm4ge25hbWU6ICdzZW5zb3InfTtcbn07XG4vKipcbipUaGlzIGZ1bmN0aW9uIGZpbGxzIHRoZSBnaXZlbiBvcHRpb25zIGJ5IHN1Y2Nlc3NpdmVseSBjaGVja2luZyBpZiBpdCB1c2VzIGEgcmVnaXN0ZXJlZCBtb2RlbCxcbiogaXQgYnVpbGRzIGFuZCBjaGVja3MgdGhlIGR5bmFtaWMgYW5kIG9ic2VydmF0aW9uIGRpbWVuc2lvbnMsIGJ1aWxkIHRoZSBzdGF0ZVByb2plY3Rpb24gaWYgb25seSBvYnNlcnZlZFByb2plY3Rpb25cbippcyBnaXZlbiwgYW5kIGluaXRpYWxpemUgZHluYW1pYy5pbml0XG4qQHBhcmFtIHtEeW5hbWljQ29uZmlnfSBvcHRpb25zLmR5bmFtaWNcbipAcGFyYW0ge09ic2VydmF0aW9uQ29uZmlnfSBvcHRpb25zLm9ic2VydmF0aW9uXG4qL1xuXG5jb25zdCBzZXR1cE1vZGVsc1BhcmFtZXRlcnMgPSBmdW5jdGlvbiAoe29ic2VydmF0aW9uLCBkeW5hbWljfSkge1xuXHRpZiAodHlwZW9mIChvYnNlcnZhdGlvbikgIT09ICdvYmplY3QnIHx8IG9ic2VydmF0aW9uID09PSBudWxsKSB7XG5cdFx0b2JzZXJ2YXRpb24gPSBidWlsZERlZmF1bHRPYnNlcnZhdGlvbihvYnNlcnZhdGlvbik7XG5cdH1cblxuXHRpZiAodHlwZW9mIChkeW5hbWljKSAhPT0gJ29iamVjdCcgfHwgZHluYW1pYyA9PT0gbnVsbCkge1xuXHRcdGR5bmFtaWMgPSBidWlsZERlZmF1bHREeW5hbWljKGR5bmFtaWMsIG9ic2VydmF0aW9uKTtcblx0fVxuXG5cdGlmICh0eXBlb2YgKG9ic2VydmF0aW9uLm5hbWUpID09PSAnc3RyaW5nJykge1xuXHRcdG9ic2VydmF0aW9uID0gbW9kZWxDb2xsZWN0aW9uLmJ1aWxkT2JzZXJ2YXRpb24ob2JzZXJ2YXRpb24pO1xuXHR9XG5cblx0aWYgKHR5cGVvZiAoZHluYW1pYy5uYW1lKSA9PT0gJ3N0cmluZycpIHtcblx0XHRkeW5hbWljID0gbW9kZWxDb2xsZWN0aW9uLmJ1aWxkRHluYW1pYyhkeW5hbWljLCBvYnNlcnZhdGlvbik7XG5cdH1cblxuXHRjb25zdCB3aXRoRGltZW5zaW9uT3B0aW9ucyA9IHNldERpbWVuc2lvbnMoe29ic2VydmF0aW9uLCBkeW5hbWljfSk7XG5cdGNvbnN0IGNoZWNrZWREaW1lbnNpb25PcHRpb25zID0gY2hlY2tEaW1lbnNpb25zKHdpdGhEaW1lbnNpb25PcHRpb25zKTtcblx0Y29uc3QgYnVpbGRTdGF0ZVByb2plY3Rpb25PcHRpb25zID0gYnVpbGRTdGF0ZVByb2plY3Rpb24oY2hlY2tlZERpbWVuc2lvbk9wdGlvbnMpO1xuXHRyZXR1cm4gZXh0ZW5kRHluYW1pY0luaXQoYnVpbGRTdGF0ZVByb2plY3Rpb25PcHRpb25zKTtcbn07XG5cbi8qKlxuKlJldHVybnMgdGhlIGNvcnJlc3BvbmRpbmcgbW9kZWwgd2l0aG91dCBhcnJheXMgYXMgdmFsdWVzIGJ1dCBvbmx5IGZ1bmN0aW9uc1xuKkBwYXJhbSB7T2JzZXJ2YXRpb25Db25maWd9IG9ic2VydmF0aW9uXG4qQHBhcmFtIHtEeW5hbWljQ29uZmlnfSBkeW5hbWljXG4qQHJldHVybnMge09ic2VydmF0aW9uQ29uZmlnLCBEeW5hbWljQ29uZmlnfSBtb2RlbCB3aXRoIHJlc3BlY3Qgb2YgdGhlIENvcmUgS2FsbWFuIEZpbHRlciBwcm9wZXJ0aWVzXG4qL1xuY29uc3QgbW9kZWxzUGFyYW1ldGVyc1RvQ29yZU9wdGlvbnMgPSBmdW5jdGlvbiAobW9kZWxUb0JlQ2hhbmdlZCkge1xuXHRjb25zdCB7b2JzZXJ2YXRpb24sIGR5bmFtaWN9ID0gbW9kZWxUb0JlQ2hhbmdlZDtcblx0cmV0dXJuIGRlZXBBc3NpZ24obW9kZWxUb0JlQ2hhbmdlZCwge1xuXHRcdG9ic2VydmF0aW9uOiB7XG5cdFx0XHRzdGF0ZVByb2plY3Rpb246IHRvRnVuY3Rpb24ocG9seW1vcnBoTWF0cml4KG9ic2VydmF0aW9uLnN0YXRlUHJvamVjdGlvbikpLFxuXHRcdFx0Y292YXJpYW5jZTogdG9GdW5jdGlvbihwb2x5bW9ycGhNYXRyaXgob2JzZXJ2YXRpb24uY292YXJpYW5jZSwge2RpbWVuc2lvbjogb2JzZXJ2YXRpb24uZGltZW5zaW9ufSkpXG5cdFx0fSxcblx0XHRkeW5hbWljOiB7XG5cdFx0XHR0cmFuc2l0aW9uOiB0b0Z1bmN0aW9uKHBvbHltb3JwaE1hdHJpeChkeW5hbWljLnRyYW5zaXRpb24pKSxcblx0XHRcdGNvdmFyaWFuY2U6IHRvRnVuY3Rpb24ocG9seW1vcnBoTWF0cml4KGR5bmFtaWMuY292YXJpYW5jZSwge2RpbWVuc2lvbjogZHluYW1pYy5kaW1lbnNpb259KSlcblx0XHR9XG5cdH0pO1xufTtcblxuY2xhc3MgS2FsbWFuRmlsdGVyIGV4dGVuZHMgQ29yZUthbG1hbkZpbHRlciB7XG5cdC8qKlxuXHQqIEBwYXJhbSB7RHluYW1pY0NvbmZpZ30gb3B0aW9ucy5keW5hbWljXG5cdCogQHBhcmFtIHtPYnNlcnZhdGlvbkNvbmZpZ30gb3B0aW9ucy5vYnNlcnZhdGlvbiB0aGUgc3lzdGVtJ3Mgb2JzZXJ2YXRpb24gbW9kZWxcblx0Ki9cblx0Y29uc3RydWN0b3Iob3B0aW9ucyA9IHt9KSB7XG5cdFx0Y29uc3QgbW9kZWxzUGFyYW1ldGVycyA9IHNldHVwTW9kZWxzUGFyYW1ldGVycyhvcHRpb25zKTtcblx0XHRjb25zdCBjb3JlT3B0aW9ucyA9IG1vZGVsc1BhcmFtZXRlcnNUb0NvcmVPcHRpb25zKG1vZGVsc1BhcmFtZXRlcnMpO1xuXG5cdFx0c3VwZXIoT2JqZWN0LmFzc2lnbih7fSwgb3B0aW9ucywgY29yZU9wdGlvbnMpKTtcblx0fVxuXG5cdGNvcnJlY3Qob3B0aW9ucykge1xuXHRcdGNvbnN0IGNvcmVPYnNlcnZhdGlvbiA9IGFycmF5VG9NYXRyaXgoe29ic2VydmF0aW9uOiBvcHRpb25zLm9ic2VydmF0aW9uLCBkaW1lbnNpb246IHRoaXMub2JzZXJ2YXRpb24uZGltZW5zaW9ufSk7XG5cdFx0cmV0dXJuIHN1cGVyLmNvcnJlY3QoT2JqZWN0LmFzc2lnbih7fSwgb3B0aW9ucywge29ic2VydmF0aW9uOiBjb3JlT2JzZXJ2YXRpb259KSk7XG5cdH1cblxuXHQvKipcblx0KlBlcmZvcm1zIHRoZSBwcmVkaWN0aW9uIGFuZCB0aGUgY29ycmVjdGlvbiBzdGVwc1xuXHQqQHBhcmFtIHtTdGF0ZX0gcHJldmlvdXNDb3JyZWN0ZWRcblx0KkBwYXJhbSB7PEFycmF5LjxOdW1iZXI+Pn0gb2JzZXJ2YXRpb25cblx0KkByZXR1cm5zIHtBcnJheS48TnVtYmVyPn0gdGhlIG1lYW4gb2YgdGhlIGNvcnJlY3Rpb25zXG5cdCovXG5cblx0ZmlsdGVyKG9wdGlvbnMpIHtcblx0XHRjb25zdCBwcmVkaWN0ZWQgPSBzdXBlci5wcmVkaWN0KG9wdGlvbnMpO1xuXHRcdHJldHVybiB0aGlzLmNvcnJlY3QoT2JqZWN0LmFzc2lnbih7fSwgb3B0aW9ucywge3ByZWRpY3RlZH0pKTtcblx0fVxuXG5cdC8qKlxuKkZpbHRlcnMgYWxsIHRoZSBvYnNlcnZhdGlvbnNcbipAcGFyYW0ge0FycmF5LjxBcnJheS48TnVtYmVyPj59IG9ic2VydmF0aW9uc1xuKkByZXR1cm5zIHtBcnJheS48QXJyYXkuPE51bWJlcj4+fSB0aGUgbWVhbiBvZiB0aGUgY29ycmVjdGlvbnNcbiovXG5cdGZpbHRlckFsbChvYnNlcnZhdGlvbnMpIHtcblx0XHRjb25zdCB7bWVhbjogbWVhbkluaXQsIGNvdmFyaWFuY2U6IGNvdmFyaWFuY2VJbml0LCBpbmRleDogaW5kZXhJbml0fSA9IHRoaXMuZHluYW1pYy5pbml0O1xuXHRcdGxldCBwcmV2aW91c0NvcnJlY3RlZCA9IG5ldyBTdGF0ZSh7XG5cdFx0XHRtZWFuOiBtZWFuSW5pdCxcblx0XHRcdGNvdmFyaWFuY2U6IGNvdmFyaWFuY2VJbml0LFxuXHRcdFx0aW5kZXg6IGluZGV4SW5pdH0pO1xuXHRcdGNvbnN0IHJlc3VsdHMgPSBbXTtcblx0XHRmb3IgKGNvbnN0IG9ic2VydmF0aW9uIG9mIG9ic2VydmF0aW9ucykge1xuXHRcdFx0Y29uc3QgcHJlZGljdGVkID0gdGhpcy5wcmVkaWN0KHtwcmV2aW91c0NvcnJlY3RlZH0pO1xuXHRcdFx0cHJldmlvdXNDb3JyZWN0ZWQgPSB0aGlzLmNvcnJlY3Qoe1xuXHRcdFx0XHRwcmVkaWN0ZWQsXG5cdFx0XHRcdG9ic2VydmF0aW9uXG5cdFx0XHR9KTtcblx0XHRcdHJlc3VsdHMucHVzaChwcmV2aW91c0NvcnJlY3RlZC5tZWFuLm1hcChtID0+IG1bMF0pKTtcblx0XHR9XG5cblx0XHRyZXR1cm4gcmVzdWx0cztcblx0fVxuXG5cdC8qKlxuXHQqIFJldHVybnMgYW4gZXN0aW1hdGlvbiBvZiB0aGUgYXN5bXB0b3RpYyBzdGF0ZSBjb3ZhcmlhbmNlIGFzIGV4cGxhaW5lZCBpbiBodHRwczovL2VuLndpa2lwZWRpYS5vcmcvd2lraS9LYWxtYW5fZmlsdGVyI0FzeW1wdG90aWNfZm9ybVxuXHQqIGluIHByYWN0aWNlIHRoaXMgY2FuIGJlIHVzZWQgYXMgYSBpbml0LmNvdmFyaWFuY2UgdmFsdWUgYnV0IGlzIHZlcnkgY29zdGZ1bCBjYWxjdWxhdGlvbiAodGhhdCdzIHdoeSB0aGlzIGlzIG5vdCBtYWRlIGJ5IGRlZmF1bHQpXG5cdCogQHBhcmFtIHtOdW1iZXJ9IFt0b2xlcmFuY2U9MWUtNl0gcmV0dXJucyB3aGVuIHRoZSBsYXN0IHZhbHVlcyBkaWZmZXJlbmNlcyBhcmUgbGVzcyB0aGFuIHRvbGVyYW5jZVxuXHQqIEByZXR1cm4gezxBcnJheS48QXJyYXkuPE51bWJlcj4+Pn0gY292YXJpYW5jZVxuXHQqL1xuXHRhc3ltcHRvdGljU3RhdGVDb3ZhcmlhbmNlKGxpbWl0SXRlcmF0aW9ucyA9IDFlMiwgdG9sZXJhbmNlID0gMWUtNikge1xuXHRcdGxldCBwcmV2aW91c0NvcnJlY3RlZCA9IHN1cGVyLmdldEluaXRTdGF0ZSgpO1xuXHRcdGxldCBwcmVkaWN0ZWQ7XG5cdFx0Y29uc3QgcmVzdWx0cyA9IFtdO1xuXHRcdGZvciAobGV0IGkgPSAwOyBpIDwgbGltaXRJdGVyYXRpb25zOyBpKyspIHtcblx0XHRcdC8vIFdlIGNyZWF0ZSBhIGZha2UgbWVhbiB0aGF0IHdpbGwgbm90IGJlIHVzZWQgaW4gb3JkZXIgdG8ga2VlcCBjb2hlcmVuY2Vcblx0XHRcdHByZWRpY3RlZCA9IG5ldyBTdGF0ZSh7XG5cdFx0XHRcdG1lYW46IG51bGwsXG5cdFx0XHRcdGNvdmFyaWFuY2U6IHN1cGVyLmdldFByZWRpY3RlZENvdmFyaWFuY2Uoe3ByZXZpb3VzQ29ycmVjdGVkfSlcblx0XHRcdH0pO1xuXHRcdFx0cHJldmlvdXNDb3JyZWN0ZWQgPSBuZXcgU3RhdGUoe1xuXHRcdFx0XHRtZWFuOiBudWxsLFxuXHRcdFx0XHRjb3ZhcmlhbmNlOiBzdXBlci5nZXRDb3JyZWN0ZWRDb3ZhcmlhbmNlKHtwcmVkaWN0ZWR9KVxuXHRcdFx0fSk7XG5cdFx0XHRyZXN1bHRzLnB1c2gocHJldmlvdXNDb3JyZWN0ZWQuY292YXJpYW5jZSk7XG5cdFx0XHRpZiAoZGlzdGFuY2VNYXQocHJldmlvdXNDb3JyZWN0ZWQuY292YXJpYW5jZSwgcmVzdWx0c1tpIC0gMV0pIDwgdG9sZXJhbmNlKSB7XG5cdFx0XHRcdHJldHVybiByZXN1bHRzW2ldO1xuXHRcdFx0fVxuXHRcdH1cblxuXHRcdHRocm93IChuZXcgRXJyb3IoJ1RoZSBzdGF0ZSBjb3ZhcmlhbmNlIGRvZXMgbm90IGNvbnZlcmdlIGFzeW1wdG90aWNhbGx5JykpO1xuXHR9XG5cblx0LyoqXG5cdCogUmV0dXJucyBhbiBlc3RpbWF0aW9uIG9mIHRoZSBhc3ltcHRvdGljIGdhaW4sIGFzIGV4cGxhaW5lZCBpbiBodHRwczovL2VuLndpa2lwZWRpYS5vcmcvd2lraS9LYWxtYW5fZmlsdGVyI0FzeW1wdG90aWNfZm9ybVxuXHQqIEBwYXJhbSB7TnVtYmVyfSBbdG9sZXJhbmNlPTFlLTZdIHJldHVybnMgd2hlbiB0aGUgbGFzdCB2YWx1ZXMgZGlmZmVyZW5jZXMgYXJlIGxlc3MgdGhhbiB0b2xlcmFuY2Vcblx0KiBAcmV0dXJuIHs8QXJyYXkuPEFycmF5LjxOdW1iZXI+Pj59IGdhaW5cblx0Ki9cblx0YXN5bXB0b3RpY0dhaW4odG9sZXJhbmNlID0gMWUtNikge1xuXHRcdGNvbnN0IGNvdmFyaWFuY2UgPSB0aGlzLmFzeW1wdG90aWNTdGF0ZUNvdmFyaWFuY2UodG9sZXJhbmNlKTtcblxuXHRcdGNvbnN0IGFzeW1wdG90aWNTdGF0ZSA9IG5ldyBTdGF0ZSh7XG5cdFx0XHQvLyBXZSBjcmVhdGUgYSBmYWtlIG1lYW4gdGhhdCB3aWxsIG5vdCBiZSB1c2VkIGluIG9yZGVyIHRvIGtlZXAgY29oZXJlbmNlXG5cdFx0XHRtZWFuOiBuZXcgQXJyYXkoY292YXJpYW5jZS5sZW5ndGgpLmZpbGwoMCkubWFwKCgpID0+IFswXSksXG5cdFx0XHRjb3ZhcmlhbmNlXG5cdFx0fSk7XG5cblx0XHRyZXR1cm4gc3VwZXIuZ2V0R2Fpbih7cHJldmlvdXNDb3JyZWN0ZWQ6IGFzeW1wdG90aWNTdGF0ZX0pO1xuXHR9XG59XG5cbm1vZHVsZS5leHBvcnRzID0gS2FsbWFuRmlsdGVyO1xuIiwiY29uc3QgZWxlbVdpc2UgPSByZXF1aXJlKCcuL2VsZW0td2lzZScpO1xuLyoqXG4qIEFkZCBtYXRyaXhlcyB0b2dldGhlclxuKiBAcGFyYW0gey4uLjxBcnJheS48QXJyYXkuPE51bWJlcj4+fSBhcmdzIGxpc3Qgb2YgbWF0cml4XG4qIEByZXR1cm5zIHtBcnJheS48QXJyYXkuPE51bWJlcj4+fSBzdW1cbiovXG5tb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uICguLi5hcmdzKSB7XG5cdHJldHVybiBlbGVtV2lzZShhcmdzLCBhcmdzMiA9PiB7XG5cdFx0cmV0dXJuIGFyZ3MyLnJlZHVjZSgoYSwgYikgPT4ge1xuXHRcdFx0aWYgKGEgPT09IG51bGwgfHwgYiA9PT0gbnVsbCkge1xuXHRcdFx0XHRyZXR1cm4gbnVsbDtcblx0XHRcdH1cblxuXHRcdFx0cmV0dXJuIGEgKyBiO1xuXHRcdH0sIDApO1xuXHR9KTtcbn07XG4iLCJjb25zdCB6ZXJvcyA9IHJlcXVpcmUoJy4vemVyb3MnKTtcblxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiAobWF0KSB7XG5cdGNvbnN0IHJlc3VsdCA9IHplcm9zKG1hdC5sZW5ndGgsIG1hdC5sZW5ndGgpO1xuXG5cdGZvciAoY29uc3QgW2ksIGVsZW1lbnRdIG9mIG1hdC5lbnRyaWVzKCkpIHtcblx0XHRyZXN1bHRbaV1baV0gPSBlbGVtZW50O1xuXHR9XG5cblx0cmV0dXJuIHJlc3VsdDtcbn07XG4iLCJjb25zdCB0cmFjZSA9IHJlcXVpcmUoJy4vdHJhY2UuanMnKTtcbmNvbnN0IHRyYW5zcG9zZSA9IHJlcXVpcmUoJy4vdHJhbnNwb3NlLmpzJyk7XG5jb25zdCBtYXRTdWIgPSByZXF1aXJlKCcuL3N1Yi5qcycpO1xuY29uc3QgbWF0TXVsID0gcmVxdWlyZSgnLi9tYXQtbXVsLmpzJyk7XG5jb25zdCBzdW0gPSByZXF1aXJlKCcuL3N1bS5qcycpO1xuXG4vLyBbRnJvYmVuaXVzIG5vcm1dKGh0dHBzOi8vZW4ud2lraXBlZGlhLm9yZy93aWtpL01hdHJpeF9ub3JtI0Zyb2Jlbml1c19ub3JtIClcbm1vZHVsZS5leHBvcnRzID0gZnVuY3Rpb24gKGFycmF5MSwgYXJyYXkyKSB7XG5cdGlmICh0eXBlb2YgKGFycmF5MSkgPT09ICd1bmRlZmluZWQnKSB7XG5cdFx0cmV0dXJuIHN1bShhcnJheTIpO1xuXHR9XG5cblx0aWYgKHR5cGVvZiAoYXJyYXkyKSA9PT0gJ3VuZGVmaW5lZCcpIHtcblx0XHRyZXR1cm4gc3VtKGFycmF5MSk7XG5cdH1cblxuXHRjb25zdCBtID0gbWF0U3ViKGFycmF5MSwgYXJyYXkyKTtcblx0Y29uc3QgcCA9IG1hdE11bCh0cmFuc3Bvc2UobSksIG0pO1xuXHRyZXR1cm4gTWF0aC5zcXJ0KHRyYWNlKHApKTtcbn07XG4iLCIvKipcbiogQGNhbGxiYWNrIGVsZW1XaXNlQ2JcbiogQHBhcmFtIHtBcnJheS48TnVtYmVyPn0gYXJyXG4qIEBwYXJhbSB7TnVtYmVyfSByb3dJZFxuKiBAcGFyYW0ge051bWJlcn0gY29sSWRcbiovXG4vKipcbiogcnVuIGEgZnVuY3Rpb24gb24gY2VsbCBwZXIgY2VsbCBmb3IgZWFjaCBNYXRyaXhlc1xuKiBAcGFyYW0gezxBcnJheS48QXJyYXkuPEFycmF5LjxOdW1iZXI+Pj59IGFyck1hdHJpeGVzIGxpc3Qgb2YgbWF0cml4ZXNcbiogQHBhcmFtIHtlbGVtV2lzZUNifSBmblxuKiBAcmV0dXJucyB7QXJyYXkuPEFycmF5LjxOdW1iZXI+Pn0gcmVzdWx0aW5nIG1hdHJpeFxuKiBAZXhhbXBsZVxuLy8gdGhpcyB3aWxsIGRvIG0xICsgbTIgKyBtMyArIG00IG9uIG1hdHJpeGVzXG5lbGVtV2lzZShbbTEsIG0yLCBtMywgbTRdLCBhcmdzMiA9PiB7XG5cdHJldHVybiBhcmdzMi5yZWR1Y2UoKGEsIGIpID0+IGEgKyBiLCAwKTtcbn0pO1xuKi9cblxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiAoYXJyYXlNYXRyaXhlcywgZm4pIHtcblx0cmV0dXJuIGFycmF5TWF0cml4ZXNbMF0ubWFwKChyb3csIHJvd0lkKSA9PiB7XG5cdFx0cmV0dXJuIHJvdy5tYXAoKGNlbGwsIGNvbElkKSA9PiB7XG5cdFx0XHRjb25zdCBhcnJheSA9IGFycmF5TWF0cml4ZXMubWFwKG0gPT4gbVtyb3dJZF1bY29sSWRdKTtcblx0XHRcdHJldHVybiBmbihhcnJheSwgcm93SWQsIGNvbElkKTtcblx0XHR9KTtcblx0fSk7XG59O1xuXG4iLCJtb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uIChzdGF0ZVNpemUpIHtcblx0Y29uc3QgaWRlbnRpdHlBcnJheSA9IFtdO1xuXHRmb3IgKGxldCBpID0gMDsgaSA8IHN0YXRlU2l6ZTsgaSsrKSB7XG5cdFx0Y29uc3Qgcm93SWRlbnRpdHkgPSBbXTtcblx0XHRmb3IgKGxldCBqID0gMDsgaiA8IHN0YXRlU2l6ZTsgaisrKSB7XG5cdFx0XHRpZiAoaSA9PT0gaikge1xuXHRcdFx0XHRyb3dJZGVudGl0eS5wdXNoKDEpO1xuXHRcdFx0fSBlbHNlIHtcblx0XHRcdFx0cm93SWRlbnRpdHkucHVzaCgwKTtcblx0XHRcdH1cblx0XHR9XG5cblx0XHRpZGVudGl0eUFycmF5LnB1c2gocm93SWRlbnRpdHkpO1xuXHR9XG5cblx0cmV0dXJuIGlkZW50aXR5QXJyYXk7XG59O1xuIiwibW9kdWxlLmV4cG9ydHMgPSB7XG5cdGFkZDogcmVxdWlyZSgnLi9hZGQuanMnKSxcblx0ZGlhZzogcmVxdWlyZSgnLi9kaWFnLmpzJyksXG5cdCdkaXN0YW5jZS1tYXQnOiByZXF1aXJlKCcuL2Rpc3RhbmNlLW1hdC5qcycpLFxuXHQnZWxlbS13aXNlJzogcmVxdWlyZSgnLi9lbGVtLXdpc2UuanMnKSxcblx0aWRlbnRpdHk6IHJlcXVpcmUoJy4vaWRlbnRpdHkuanMnKSxcblx0aW52ZXJ0OiByZXF1aXJlKCcuL2ludmVydC5qcycpLFxuXHQnbWF0LW11bCc6IHJlcXVpcmUoJy4vbWF0LW11bC5qcycpLFxuXHQncGFkLXdpdGgtemVyb3MnOiByZXF1aXJlKCcuL3BhZC13aXRoLXplcm9zLmpzJyksXG5cdHN1YjogcmVxdWlyZSgnLi9zdWIuanMnKSxcblx0J3N1Yi1zcXVhcmUtbWF0cml4JzogcmVxdWlyZSgnLi9zdWItc3F1YXJlLW1hdHJpeC5qcycpLFxuXHRzdW06IHJlcXVpcmUoJy4vc3VtLmpzJyksXG5cdHRyYWNlOiByZXF1aXJlKCcuL3RyYWNlLmpzJyksXG5cdHRyYW5zcG9zZTogcmVxdWlyZSgnLi90cmFuc3Bvc2UuanMnKSxcblx0emVyb3M6IHJlcXVpcmUoJy4vemVyb3MuanMnKVxufTtcbiIsImNvbnN0IG1hdHJpeEludmVyc2UgPSByZXF1aXJlKCdtYXRyaXgtaW52ZXJzZScpO1xuXG5tb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uIChtKSB7XG5cdHJldHVybiBtYXRyaXhJbnZlcnNlKG0pO1xufTtcbiIsIi8qKlxuKiBNdWx0aXBseSAyIG1hdHJpeGVzIHRvZ2V0aGVyXG4qIEBwYXJhbSB7PEFycmF5LjxBcnJheS48TnVtYmVyPj59IG0xXG4qIEBwYXJhbSB7PEFycmF5LjxBcnJheS48TnVtYmVyPj59IG0yXG4qIEByZXR1cm5zIHtBcnJheS48QXJyYXkuPE51bWJlcj4+fVxuKi9cbm1vZHVsZS5leHBvcnRzID0gZnVuY3Rpb24gKG0xLCBtMikge1xuXHQvLyBDb25zb2xlLmxvZyh7bTEsIG0yfSk7XG5cdGNvbnN0IHJlc3VsdCA9IFtdO1xuXHRmb3IgKGxldCBpID0gMDsgaSA8IG0xLmxlbmd0aDsgaSsrKSB7XG5cdFx0cmVzdWx0W2ldID0gW107XG5cdFx0Zm9yIChsZXQgaiA9IDA7IGogPCBtMlswXS5sZW5ndGg7IGorKykge1xuXHRcdFx0bGV0IHN1bSA9IDA7XG5cdFx0XHRsZXQgaXNOdWxsID0gZmFsc2U7XG5cdFx0XHRmb3IgKGxldCBrID0gMDsgayA8IG0xWzBdLmxlbmd0aDsgaysrKSB7XG5cdFx0XHRcdGlmICgobTFbaV1ba10gPT09IG51bGwgJiYgbTJba11bal0gIT09IDApIHx8IChtMltrXVtqXSA9PT0gbnVsbCAmJiBtMVtpXVtrXSAhPT0gMCkpIHtcblx0XHRcdFx0XHRpc051bGwgPSB0cnVlO1xuXHRcdFx0XHR9XG5cblx0XHRcdFx0c3VtICs9IG0xW2ldW2tdICogbTJba11bal07XG5cdFx0XHR9XG5cblx0XHRcdHJlc3VsdFtpXVtqXSA9IGlzTnVsbCA/IG51bGwgOiBzdW07XG5cdFx0fVxuXHR9XG5cblx0cmV0dXJuIHJlc3VsdDtcbn07XG4iLCIvKipcbipUaGlzIGZ1bmN0aW9uIHJldHVybnMgdGhlIHN0YXRlUHJvamVjdGlvbiBwYWRlZCB3aXRoIHplcm9zIHdpdGggcmVzcGVjdCB0byBhIGdpdmVuXG4qb2JzZXJ2ZWRQcm9qZWN0aW9uXG4qQHBhcmFtIHtBcnJheS48TnVtYmVyPiB8IEFycmF5LjxBcnJheS48TnVtYmVyPj59IGFycmF5IHRoZSBhcnJheSB3ZSBuZWVkIHRvIHBhZFxuKkBwYXJhbSB7TnVtYmVyfSBkaW1lbnNpb24gaW4gb3VyIGNhc2UsIHRoZSBkeW5hbWljIGRpbWVuc2lvblxuKkByZXR1cm5zIHtBcnJheS48TnVtYmVyPiB8IEFycmF5LjxBcnJheS48TnVtYmVyPj59IHBhZGVkIGFycmF5XG4qL1xubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiAoYXJyYXksIHtkaW1lbnNpb259KSB7XG5cdGNvbnN0IGwxID0gYXJyYXkubGVuZ3RoO1xuXHRjb25zdCBsID0gYXJyYXlbMF0ubGVuZ3RoO1xuXHRjb25zdCByZXN1bHQgPSBhcnJheS5tYXAoYSA9PiBhLmNvbmNhdCgpKTtcblxuXHRpZiAoZGltZW5zaW9uIDwgbCkge1xuXHRcdHRocm93IChuZXcgVHlwZUVycm9yKGBEeW5hbWljIGRpbWVuc2lvbiAke2RpbWVuc2lvbn0gZG9lcyBub3QgbWF0Y2ggd2l0aCBvYnNlcnZlZFByb2plY3Rpb24gJHtsfWApKTtcblx0fVxuXG5cdGZvciAobGV0IGkgPSAwOyBpIDwgbDE7IGkrKykge1xuXHRcdGZvciAobGV0IGogPSAwOyBqIDwgZGltZW5zaW9uIC0gbDsgaisrKSB7XG5cdFx0XHRyZXN1bHRbaV0ucHVzaCgwKTtcblx0XHR9XG5cdH1cblxuXHRyZXR1cm4gcmVzdWx0O1xufTtcbiIsIm1vZHVsZS5leHBvcnRzID0gKG1hdCwgb2JzSW5kZXhlcykgPT4ge1xuXHRyZXR1cm4gb2JzSW5kZXhlcy5tYXAoczEgPT4gb2JzSW5kZXhlcy5tYXAoczIgPT4gbWF0W3MxXVtzMl0pKTtcbn07XG4iLCJjb25zdCBlbGVtV2lzZSA9IHJlcXVpcmUoJy4vZWxlbS13aXNlJyk7XG5cbm1vZHVsZS5leHBvcnRzID0gZnVuY3Rpb24gKC4uLmFyZ3MpIHtcblx0cmV0dXJuIGVsZW1XaXNlKGFyZ3MsIChbYSwgYl0pID0+IGEgLSBiKTtcbn07XG4iLCIvLyBTdW0gYWxsIHRoZSB0ZXJtcyBvZiBhIGdpdmVuIG1hdHJpeFxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiAoYXJyYXkpIHtcblx0bGV0IHMgPSAwO1xuXHRmb3IgKGxldCBpID0gMDsgaSA8IGFycmF5Lmxlbmd0aDsgaSsrKSB7XG5cdFx0Zm9yIChsZXQgaiA9IDA7IGogPCBhcnJheS5sZW5ndGg7IGorKykge1xuXHRcdFx0cyArPSBhcnJheVtpXVtqXTtcblx0XHR9XG5cdH1cblxuXHRyZXR1cm4gcztcbn07XG4iLCJtb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uIChhcnJheSkge1xuXHRsZXQgZGlhZyA9IDA7XG5cdGZvciAoY29uc3QgW3JvdywgZWxlbWVudF0gb2YgYXJyYXkuZW50cmllcygpKSB7XG5cdFx0ZGlhZyArPSBlbGVtZW50W3Jvd107XG5cdH1cblxuXHRyZXR1cm4gZGlhZztcbn07XG4iLCJtb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uIChhcnJheSkge1xuXHRyZXR1cm4gYXJyYXlbMF0ubWFwKChjb2wsIGkpID0+IGFycmF5Lm1hcChyb3cgPT4gcm93W2ldKSk7XG59O1xuIiwibW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiAocm93cywgY29scykge1xuXHRyZXR1cm4gbmV3IEFycmF5KHJvd3MpLmZpbGwoMSkubWFwKCgpID0+IG5ldyBBcnJheShjb2xzKS5maWxsKDApKTtcbn07XG4iLCJjb25zdCByZWdpc3RlcmVkRHluYW1pY01vZGVscyA9IHtcblx0J2NvbnN0YW50LXBvc2l0aW9uJzogcmVxdWlyZSgnLi4vbGliL2R5bmFtaWMvY29uc3RhbnQtcG9zaXRpb24uanMnKSxcblx0J2NvbnN0YW50LXNwZWVkJzogcmVxdWlyZSgnLi4vbGliL2R5bmFtaWMvY29uc3RhbnQtc3BlZWQuanMnKSxcblx0J2NvbnN0YW50LWFjY2VsZXJhdGlvbic6IHJlcXVpcmUoJy4uL2xpYi9keW5hbWljL2NvbnN0YW50LWFjY2VsZXJhdGlvbi5qcycpXG59O1xuY29uc3QgcmVnaXN0ZXJlZE9ic2VydmF0aW9uTW9kZWxzID0ge1xuXHRzZW5zb3I6IHJlcXVpcmUoJy4uL2xpYi9vYnNlcnZhdGlvbi9zZW5zb3IuanMnKVxufTtcblxuLyoqXG4qUmVnaXN0ZXJPYnNlcnZhdGlvbiBlbmFibGVzIHRvIGNyZWF0ZSBhIG5ldyBvYnNlcnZhdGlvbiBtb2RlbCBhbmQgc3RvY2sgaXRcbiogQHBhcmFtIHtTdHJpbmd9IG5hbWVcbiogQGNhbGxiYWNrIGZuIHRoZSBmdW5jdGlvbiBjb3JyZXNwb25kaW5nIHRvIHRoZSBkZXNpcmVkIG1vZGVsXG4qL1xuXG4vKipcbipyZWdpc3RlckR5bmFtaWMgZW5hYmxlcyB0byBjcmVhdGUgYSBuZXcgZHluYW1pYyBtb2RlbCBhbmQgc3RvY2tzIGl0XG4qIEBwYXJhbSB7U3RyaW5nfSBuYW1lXG4qIEBjYWxsYmFjayBmbiB0aGUgZnVuY3Rpb24gY29ycmVzcG9uZGluZyB0byB0aGUgZGVzaXJlZCBtb2RlbFxuKi9cblxuLyoqXG4qYnVpbGRPYnNlcnZhdGlvbiBlbmFibGVzIHRvIGJ1aWxkIGEgbW9kZWwgZ2l2ZW4gYW4gb2JzZXJ2YXRpb24gY29uZmlndXJhdGlvblxuKiBAcGFyYW0ge09ic2VydmF0aW9uQ29uZmlnfSBvYnNlcnZhdGlvblxuKiBAcmV0dXJucyB7T2JzZXJ2YXRpb25Db25maWd9IHRoZSBjb25maWd1cmF0aW9uIHdpdGggcmVzcGVjdCB0byB0aGUgbW9kZWxcbiovXG5cbi8qKlxuKmJ1aWxkRHluYW1pYyBlbmFibGVzIHRvIGJ1aWxkIGEgbW9kZWwgZ2l2ZW4gZHluYW1pYyBhbmQgb2JzZXJ2YXRpb24gY29uZmlndXJhdGlvbnNcbiogQHBhcmFtIHtEeW5hbWljQ29uZmlnfSBkeW5hbWljXG4qIEBwYXJhbSB7T2JzZXJ2YXRpb25Db25maWd9IG9ic2VydmF0aW9uXG4qIEByZXR1cm5zIHtEeW5hbWljQ29uZmlnfSB0aGUgZHluYW1pYyBjb25maWd1cmF0aW9uIHdpdGggcmVzcGVjdCB0byB0aGUgbW9kZWxcbiovXG5cbm1vZHVsZS5leHBvcnRzID0ge1xuXHRyZWdpc3Rlck9ic2VydmF0aW9uOiAobmFtZSwgZm4pID0+IHtcblx0XHRyZWdpc3RlcmVkT2JzZXJ2YXRpb25Nb2RlbHNbbmFtZV0gPSBmbjtcblx0fSxcblx0cmVnaXN0ZXJEeW5hbWljOiAobmFtZSwgZm4pID0+IHtcblx0XHRyZWdpc3RlcmVkRHluYW1pY01vZGVsc1tuYW1lXSA9IGZuO1xuXHR9LFxuXHRidWlsZE9ic2VydmF0aW9uOiBvYnNlcnZhdGlvbiA9PiB7XG5cdFx0aWYgKCFyZWdpc3RlcmVkT2JzZXJ2YXRpb25Nb2RlbHNbb2JzZXJ2YXRpb24ubmFtZV0pIHtcblx0XHRcdHRocm93IChuZXcgRXJyb3IoYFRoZSBwcm92aWRlZCBvYnNlcnZhdGlvbiBtb2RlbCBuYW1lICgke29ic2VydmF0aW9uLm5hbWV9KSBpcyBub3QgcmVnaXN0ZXJlZGApKTtcblx0XHR9XG5cblx0XHRyZXR1cm4gcmVnaXN0ZXJlZE9ic2VydmF0aW9uTW9kZWxzW29ic2VydmF0aW9uLm5hbWVdKG9ic2VydmF0aW9uKTtcblx0fSxcblx0YnVpbGREeW5hbWljOiAoZHluYW1pYywgb2JzZXJ2YXRpb24pID0+IHtcblx0XHRpZiAoIXJlZ2lzdGVyZWREeW5hbWljTW9kZWxzW2R5bmFtaWMubmFtZV0pIHtcblx0XHRcdHRocm93IChuZXcgRXJyb3IoYFRoZSBwcm92aWRlZCBkeW5hbWljIG1vZGVsICgke2R5bmFtaWMubmFtZX0pIG5hbWUgaXMgbm90IHJlZ2lzdGVyZWRgKSk7XG5cdFx0fVxuXG5cdFx0cmV0dXJuIHJlZ2lzdGVyZWREeW5hbWljTW9kZWxzW2R5bmFtaWMubmFtZV0oZHluYW1pYywgb2JzZXJ2YXRpb24pO1xuXHR9XG59O1xuIiwiY29uc3QgaWRlbnRpdHkgPSByZXF1aXJlKCcuLi9saW5hbGdlYnJhL2lkZW50aXR5LmpzJyk7XG5jb25zdCBwb2x5bW9ycGhNYXRyaXggPSByZXF1aXJlKCcuLi91dGlscy9wb2x5bW9ycGgtbWF0cml4LmpzJyk7XG5jb25zdCBjaGVja01hdHJpeCA9IHJlcXVpcmUoJy4uL3V0aWxzL2NoZWNrLW1hdHJpeC5qcycpO1xuXG4vKipcbiogQHBhcmFtIHtOdW1iZXJ9IHNlbnNvckRpbWVuc2lvblxuKiBAcGFyYW0ge0NvdmFyaWFuY2VQYXJhbX0gc2Vuc29yQ292YXJpYW5jZVxuKiBAcGFyYW0ge051bWJlcn0gblNlbnNvcnNcbiogQHJldHVybnMge09ic2VydmF0aW9uQ29uZmlnfVxuKi9cblxuY29uc3QgY29weSA9IG1hdCA9PiBtYXQubWFwKGEgPT4gYS5jb25jYXQoKSk7XG5cbm1vZHVsZS5leHBvcnRzID0gZnVuY3Rpb24gKG9wdGlvbnMpIHtcblx0Y29uc3Qge3NlbnNvckRpbWVuc2lvbiA9IDEsIHNlbnNvckNvdmFyaWFuY2UgPSAxLCBuU2Vuc29ycyA9IDF9ID0gb3B0aW9ucztcblx0Y29uc3Qgc2Vuc29yQ292YXJpYW5jZUZvcm1hdHRlZCA9IHBvbHltb3JwaE1hdHJpeChzZW5zb3JDb3ZhcmlhbmNlLCB7ZGltZW5zaW9uOiBzZW5zb3JEaW1lbnNpb259KTtcblx0Y2hlY2tNYXRyaXgoc2Vuc29yQ292YXJpYW5jZUZvcm1hdHRlZCwgW3NlbnNvckRpbWVuc2lvbiwgc2Vuc29yRGltZW5zaW9uXSwgJ29ic2VydmF0aW9uLnNlbnNvckNvdmFyaWFuY2UnKTtcblx0Y29uc3Qgb25lU2Vuc29yT2JzZXJ2ZWRQcm9qZWN0aW9uID0gaWRlbnRpdHkoc2Vuc29yRGltZW5zaW9uKTtcblx0bGV0IGNvbmNhdGVuYXRlZE9ic2VydmVkUHJvamVjdGlvbiA9IFtdO1xuXHRjb25zdCBkaW1lbnNpb24gPSBzZW5zb3JEaW1lbnNpb24gKiBuU2Vuc29ycztcblx0Y29uc3QgY29uY2F0ZW5hdGVkQ292YXJpYW5jZSA9IGlkZW50aXR5KGRpbWVuc2lvbik7XG5cdGZvciAobGV0IGkgPSAwOyBpIDwgblNlbnNvcnM7IGkrKykge1xuXHRcdGNvbmNhdGVuYXRlZE9ic2VydmVkUHJvamVjdGlvbiA9IGNvbmNhdGVuYXRlZE9ic2VydmVkUHJvamVjdGlvbi5jb25jYXQoY29weShvbmVTZW5zb3JPYnNlcnZlZFByb2plY3Rpb24pKTtcblxuXHRcdHNlbnNvckNvdmFyaWFuY2VGb3JtYXR0ZWQuZm9yRWFjaCgociwgckluZGV4KSA9PiByLmZvckVhY2goKGMsIGNJbmRleCkgPT4ge1xuXHRcdFx0Y29uY2F0ZW5hdGVkQ292YXJpYW5jZVtySW5kZXggKyAoaSAqIHNlbnNvckRpbWVuc2lvbildW2NJbmRleCArIChpICogc2Vuc29yRGltZW5zaW9uKV0gPSBjO1xuXHRcdH0pKTtcblx0fVxuXG5cdHJldHVybiBPYmplY3QuYXNzaWduKHt9LCBvcHRpb25zLCB7XG5cdFx0ZGltZW5zaW9uLFxuXHRcdG9ic2VydmVkUHJvamVjdGlvbjogY29uY2F0ZW5hdGVkT2JzZXJ2ZWRQcm9qZWN0aW9uLFxuXHRcdGNvdmFyaWFuY2U6IGNvbmNhdGVuYXRlZENvdmFyaWFuY2Vcblx0fSk7XG59O1xuIiwiY29uc3QgcGFkV2l0aFplcm9zID0gcmVxdWlyZSgnLi4vbGluYWxnZWJyYS9wYWQtd2l0aC16ZXJvcy5qcycpO1xuY29uc3QgaWRlbnRpdHkgPSByZXF1aXJlKCcuLi9saW5hbGdlYnJhL2lkZW50aXR5LmpzJyk7XG4vKipcbipCdWlsZHMgdGhlIHN0YXRlUHJvamVjdGlvbiBnaXZlbiBhbiBvYnNlcnZlZFByb2plY3Rpb25cbipAcGFyYW0ge09ic2VydmF0aW9uQ29uZmlnfSBvYnNlcnZhdGlvblxuKkBwYXJhbSB7RHluYW1pY0NvbmZpZ30gZHluYW1pY1xuKkByZXR1cm5zIHtPYnNlcnZhdGlvbkNvbmZpZywgRHluYW1pY0NvbmZpZ30gdGhlIG1vZGVsIGNvbnRhaW5pbmcgdGhlIGNyZWF0ZWQgc3RhdGVQcm9qZWN0aW9uXG4qL1xuXG5tb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uICh7b2JzZXJ2YXRpb24sIGR5bmFtaWN9KSB7XG5cdGNvbnN0IHtvYnNlcnZlZFByb2plY3Rpb24sIHN0YXRlUHJvamVjdGlvbn0gPSBvYnNlcnZhdGlvbjtcblx0Y29uc3Qgb2JzZXJ2YXRpb25EaW1lbnNpb24gPSBvYnNlcnZhdGlvbi5kaW1lbnNpb247XG5cdGNvbnN0IGR5bmFtaWNEaW1lbnNpb24gPSBkeW5hbWljLmRpbWVuc2lvbjtcblx0aWYgKG9ic2VydmVkUHJvamVjdGlvbiAmJiBzdGF0ZVByb2plY3Rpb24pIHtcblx0XHR0aHJvdyAobmV3IFR5cGVFcnJvcignWW91IGNhbm5vdCB1c2UgYm90aCBvYnNlcnZlZFByb2plY3Rpb24gYW5kIHN0YXRlUHJvamVjdGlvbicpKTtcblx0fVxuXG5cdGlmIChvYnNlcnZlZFByb2plY3Rpb24pIHtcblx0XHRjb25zdCBzdGF0ZVByb2plY3Rpb24gPSBwYWRXaXRoWmVyb3Mob2JzZXJ2ZWRQcm9qZWN0aW9uLCB7ZGltZW5zaW9uOiBkeW5hbWljRGltZW5zaW9ufSk7XG5cdFx0cmV0dXJuIHtcblx0XHRcdG9ic2VydmF0aW9uOiBPYmplY3QuYXNzaWduKHt9LCBvYnNlcnZhdGlvbiwge1xuXHRcdFx0XHRzdGF0ZVByb2plY3Rpb25cblx0XHRcdH0pLFxuXHRcdFx0ZHluYW1pY1xuXHRcdH07XG5cdH1cblxuXHRpZiAob2JzZXJ2YXRpb25EaW1lbnNpb24gJiYgZHluYW1pY0RpbWVuc2lvbiAmJiAhc3RhdGVQcm9qZWN0aW9uKSB7XG5cdFx0Y29uc3Qgb2JzZXJ2YXRpb25NYXRyaXggPSBpZGVudGl0eShvYnNlcnZhdGlvbkRpbWVuc2lvbik7XG5cdFx0cmV0dXJuIHtcblx0XHRcdG9ic2VydmF0aW9uOiBPYmplY3QuYXNzaWduKHt9LCBvYnNlcnZhdGlvbiwge1xuXHRcdFx0XHRzdGF0ZVByb2plY3Rpb246IHBhZFdpdGhaZXJvcyhvYnNlcnZhdGlvbk1hdHJpeCwge2RpbWVuc2lvbjogZHluYW1pY0RpbWVuc2lvbn0pXG5cdFx0XHR9KSxcblx0XHRcdGR5bmFtaWNcblx0XHR9O1xuXHR9XG5cblx0cmV0dXJuIHtvYnNlcnZhdGlvbiwgZHluYW1pY307XG59O1xuIiwiLyoqXG4qVmVyaWZpZXMgdGhhdCBkeW5hbWljLmRpbWVuc2lvbiBhbmQgb2JzZXJ2YXRpb24uZGltZW5zaW9uIGFyZSBzZXRcbipAcGFyYW0ge09ic2VydmF0aW9uQ29uZmlnfSBvYnNlcnZhdGlvblxuKkBwYXJhbSB7RHluYW1pY0NvbmZpZ30gZHluYW1pY1xuKi9cblxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiAoe29ic2VydmF0aW9uLCBkeW5hbWljfSkge1xuXHRjb25zdCBkeW5hbWljRGltZW5zaW9uID0gZHluYW1pYy5kaW1lbnNpb247XG5cdGNvbnN0IG9ic2VydmF0aW9uRGltZW5zaW9uID0gb2JzZXJ2YXRpb24uZGltZW5zaW9uO1xuXHRpZiAoIWR5bmFtaWNEaW1lbnNpb24gfHwgIW9ic2VydmF0aW9uRGltZW5zaW9uKSB7XG5cdFx0dGhyb3cgKG5ldyBUeXBlRXJyb3IoJ0RpbWVuc2lvbiBpcyBub3Qgc2V0JykpO1xuXHR9XG5cblx0cmV0dXJuIHtvYnNlcnZhdGlvbiwgZHluYW1pY307XG59O1xuIiwiY29uc3QgZGlhZyA9IHJlcXVpcmUoJy4uL2xpbmFsZ2VicmEvZGlhZy5qcycpO1xuXG4vKipcbipJbml0aWFsaXplcyB0aGUgZHluYW1pYy5pbml0IHdoZW4gbm90IGdpdmVuXG4qQHBhcmFtIHtPYnNlcnZhdGlvbkNvbmZpZ30gb2JzZXJ2YXRpb25cbipAcGFyYW0ge0R5bmFtaWNDb25maWd9IGR5bmFtaWNcbipAcmV0dXJucyB7T2JzZXJ2YXRpb25Db25maWcsIER5bmFtaWNDb25maWd9XG4qL1xuXG5tb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uICh7b2JzZXJ2YXRpb24sIGR5bmFtaWN9KSB7XG5cdGlmICghZHluYW1pYy5pbml0KSB7XG5cdFx0Y29uc3QgaHVnZSA9IDFlNjtcblx0XHRjb25zdCBkeW5hbWljRGltZW5zaW9uID0gZHluYW1pYy5kaW1lbnNpb247XG5cdFx0Y29uc3QgbWVhbkFycmF5ID0gbmV3IEFycmF5KGR5bmFtaWNEaW1lbnNpb24pLmZpbGwoMCk7XG5cdFx0Y29uc3QgY292YXJpYW5jZUFycmF5ID0gbmV3IEFycmF5KGR5bmFtaWNEaW1lbnNpb24pLmZpbGwoaHVnZSk7XG5cdFx0Y29uc3Qgd2l0aEluaXRPcHRpb25zID0ge1xuXHRcdFx0b2JzZXJ2YXRpb24sXG5cdFx0XHRkeW5hbWljOiBPYmplY3QuYXNzaWduKHt9LCBkeW5hbWljLCB7XG5cdFx0XHRcdGluaXQ6IHtcblx0XHRcdFx0XHRtZWFuOiBtZWFuQXJyYXkubWFwKGVsZW1lbnQgPT4gW2VsZW1lbnRdKSxcblx0XHRcdFx0XHRjb3ZhcmlhbmNlOiBkaWFnKGNvdmFyaWFuY2VBcnJheSksXG5cdFx0XHRcdFx0aW5kZXg6IC0xXG5cdFx0XHRcdH1cblx0XHRcdH0pXG5cdFx0fTtcblx0XHRyZXR1cm4gd2l0aEluaXRPcHRpb25zO1xuXHR9XG5cblx0cmV0dXJuIHtvYnNlcnZhdGlvbiwgZHluYW1pY307XG59O1xuIiwiLyoqXG4qVmVyaWZpZXMgdGhhdCBkaW1lbnNpb25zIGFyZSBtYXRjaGluZyBhbmQgc2V0IGR5bmFtaWMuZGltZW5zaW9uIGFuZCBvYnNlcnZhdGlvbi5kaW1lbnNpb25cbiogd2l0aCByZXNwZWN0IG9mIHN0YXRlUHJvamVjdGlvbiBhbmQgdHJhbnNpdGlvbiBkaW1lbnNpb25zXG4qQHBhcmFtIHtPYnNlcnZhdGlvbkNvbmZpZ30gb2JzZXJ2YXRpb25cbipAcGFyYW0ge0R5bmFtaWNDb25maWd9IGR5bmFtaWNcbipAcmV0dXJucyB7T2JzZXJ2YXRpb25Db25maWcsIER5bmFtaWNDb25maWd9XG4qL1xuXG5tb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uICh7b2JzZXJ2YXRpb24sIGR5bmFtaWN9KSB7XG5cdGNvbnN0IHtzdGF0ZVByb2plY3Rpb259ID0gb2JzZXJ2YXRpb247XG5cdGNvbnN0IHt0cmFuc2l0aW9ufSA9IGR5bmFtaWM7XG5cdGNvbnN0IGR5bmFtaWNEaW1lbnNpb24gPSBkeW5hbWljLmRpbWVuc2lvbjtcblx0Y29uc3Qgb2JzZXJ2YXRpb25EaW1lbnNpb24gPSBvYnNlcnZhdGlvbi5kaW1lbnNpb247XG5cblx0aWYgKGR5bmFtaWNEaW1lbnNpb24gJiYgb2JzZXJ2YXRpb25EaW1lbnNpb24gJiYgQXJyYXkuaXNBcnJheShzdGF0ZVByb2plY3Rpb24pICYmIChkeW5hbWljRGltZW5zaW9uICE9PSBzdGF0ZVByb2plY3Rpb25bMF0ubGVuZ3RoIHx8IG9ic2VydmF0aW9uRGltZW5zaW9uICE9PSBzdGF0ZVByb2plY3Rpb24ubGVuZ3RoKSkge1xuXHRcdHRocm93IChuZXcgVHlwZUVycm9yKCdzdGF0ZVByb2plY3Rpb24gZGltZW5zaW9ucyBub3QgbWF0Y2hpbmcgd2l0aCBvYnNlcnZhdGlvbiBhbmQgZHluYW1pYyBkaW1lbnNpb25zJykpO1xuXHR9XG5cblx0aWYgKGR5bmFtaWNEaW1lbnNpb24gJiYgQXJyYXkuaXNBcnJheSh0cmFuc2l0aW9uKSAmJiBkeW5hbWljRGltZW5zaW9uICE9PSB0cmFuc2l0aW9uLmxlbmd0aCkge1xuXHRcdHRocm93IChuZXcgVHlwZUVycm9yKCd0cmFuc2l0aW9uIGRpbWVuc2lvbiBub3QgbWF0Y2hpbmcgd2l0aCBkeW5hbWljIGRpbWVuc2lvbicpKTtcblx0fVxuXG5cdGlmIChBcnJheS5pc0FycmF5KHN0YXRlUHJvamVjdGlvbikpIHtcblx0XHRyZXR1cm4ge1xuXHRcdFx0b2JzZXJ2YXRpb246IE9iamVjdC5hc3NpZ24oe30sIG9ic2VydmF0aW9uLCB7XG5cdFx0XHRcdGRpbWVuc2lvbjogc3RhdGVQcm9qZWN0aW9uLmxlbmd0aFxuXHRcdFx0fSksXG5cdFx0XHRkeW5hbWljOiBPYmplY3QuYXNzaWduKHt9LCBkeW5hbWljLCB7XG5cdFx0XHRcdGRpbWVuc2lvbjogc3RhdGVQcm9qZWN0aW9uWzBdLmxlbmd0aFxuXHRcdFx0fSlcblx0XHR9O1xuXHR9XG5cblx0aWYgKEFycmF5LmlzQXJyYXkodHJhbnNpdGlvbikpIHtcblx0XHRyZXR1cm4ge1xuXHRcdFx0b2JzZXJ2YXRpb24sXG5cdFx0XHRkeW5hbWljOiBPYmplY3QuYXNzaWduKHt9LCBkeW5hbWljLCB7XG5cdFx0XHRcdGRpbWVuc2lvbjogdHJhbnNpdGlvbi5sZW5ndGhcblx0XHRcdH0pXG5cdFx0fTtcblx0fVxuXG5cdHJldHVybiB7b2JzZXJ2YXRpb24sIGR5bmFtaWN9O1xufTtcbiIsImNvbnN0IHN1YiA9IHJlcXVpcmUoJy4vbGluYWxnZWJyYS9zdWIuanMnKTtcbmNvbnN0IHRyYW5zcG9zZSA9IHJlcXVpcmUoJy4vbGluYWxnZWJyYS90cmFuc3Bvc2UuanMnKTtcbmNvbnN0IG1hdE11bCA9IHJlcXVpcmUoJy4vbGluYWxnZWJyYS9tYXQtbXVsLmpzJyk7XG5jb25zdCBpbnZlcnQgPSByZXF1aXJlKCcuL2xpbmFsZ2VicmEvaW52ZXJ0LmpzJyk7XG5jb25zdCBlbGVtV2lzZSA9IHJlcXVpcmUoJy4vbGluYWxnZWJyYS9lbGVtLXdpc2UuanMnKTtcbmNvbnN0IHN1YlNxdWFyZU1hdHJpeCA9IHJlcXVpcmUoJy4vbGluYWxnZWJyYS9zdWItc3F1YXJlLW1hdHJpeCcpO1xuY29uc3QgYXJyYXlUb01hdHJpeCA9IHJlcXVpcmUoJy4vdXRpbHMvYXJyYXktdG8tbWF0cml4LmpzJyk7XG5cbmNvbnN0IGNoZWNrTWF0cml4ID0gcmVxdWlyZSgnLi91dGlscy9jaGVjay1tYXRyaXguanMnKTtcbmNvbnN0IGNoZWNrQ292YXJpYW5jZSA9IHJlcXVpcmUoJy4vdXRpbHMvY2hlY2stY292YXJpYW5jZScpO1xuXG4vKipcbiAqIEBjbGFzc1xuICogQ2xhc3MgcmVwcmVzZW50aW5nIGEgbXVsdGkgZGltZW5zaW9ubmFsIGdhdXNzaWFuLCB3aXRoIGhpcyBtZWFuIGFuZCBoaXMgY292YXJpYW5jZVxuICogQHByb3BlcnR5IHtOdW1iZXJ9IFtpbmRleD0wXSB0aGUgaW5kZXggb2YgdGhlIFN0YXRlIGluIHRoZSBwcm9jZXNzLCB0aGlzIGlzIG5vdCBtYW5kYXRvcnkgZm9yIHNpbXBsZSBLYWxtYW4gRmlsdGVyLCBidXQgaXMgbmVlZGVkIGZvciBtb3N0IG9mIHRoZSB1c2UgY2FzZSBvZiBleHRlbmRlZCBrYWxtYW4gZmlsdGVyXG4gKiBAcHJvcGVydHkge0FycmF5LjxBcnJheS48TnVtYmVyPj59IGNvdmFyaWFuY2Ugc3F1YXJlIG1hdHJpeCBvZiBzaXplIGRpbWVuc2lvblxuICogQHByb3BlcnR5IHtBcnJheS48QXJyYXk8TnVtYmVyPj59IG1lYW4gY29sdW1uIG1hdHJpeCBvZiBzaXplIGRpbWVuc2lvbiB4IDFcbiAqL1xuY2xhc3MgU3RhdGUge1xuXHRjb25zdHJ1Y3Rvcih7bWVhbiwgY292YXJpYW5jZSwgaW5kZXh9KSB7XG5cdFx0dGhpcy5tZWFuID0gbWVhbjtcblx0XHR0aGlzLmNvdmFyaWFuY2UgPSBjb3ZhcmlhbmNlO1xuXHRcdHRoaXMuaW5kZXggPSBpbmRleDtcblx0fVxuXG5cdC8qKlxuXHQqIENoZWNrIHRoZSBjb25zaXN0ZW5jeSBvZiB0aGUgU3RhdGVcblx0Ki9cblx0Y2hlY2sob3B0aW9ucykge1xuXHRcdHRoaXMuY29uc3RydWN0b3IuY2hlY2sodGhpcywgb3B0aW9ucyk7XG5cdH1cblxuXHQvKipcblx0KiBDaGVjayB0aGUgY29uc2lzdGVuY3kgb2YgdGhlIFN0YXRlJ3MgYXR0cmlidXRlc1xuXHQqL1xuXG5cdHN0YXRpYyBjaGVjayhzdGF0ZSwge2RpbWVuc2lvbiA9IG51bGwsIHRpdGxlID0gbnVsbCwgZWlnZW59ID0ge30pIHtcblx0XHRpZiAoIShzdGF0ZSBpbnN0YW5jZW9mIFN0YXRlKSkge1xuXHRcdFx0dGhyb3cgKG5ldyBUeXBlRXJyb3IoXG5cdFx0XHRcdCdUaGUgYXJndW1lbnQgaXMgbm90IGEgc3RhdGUgXFxuJyArXG4gICAgICAgICdUaXBzOiBtYXliZSB5b3UgYXJlIHVzaW5nIDIgZGlmZmVyZW50IHZlcnNpb24gb2Yga2FsbWFuLWZpbHRlciBpbiB5b3VyIG5wbSBkZXBzIHRyZWUnXG5cdFx0XHQpKTtcblx0XHR9XG5cblx0XHRjb25zdCB7bWVhbiwgY292YXJpYW5jZX0gPSBzdGF0ZTsgLy8gSW5kZXhcblx0XHRjb25zdCBtZWFuRGltZW5zaW9uID0gbWVhbi5sZW5ndGg7XG5cdFx0aWYgKHR5cGVvZiAoZGltZW5zaW9uKSA9PT0gJ251bWJlcicgJiYgbWVhbkRpbWVuc2lvbiAhPT0gZGltZW5zaW9uKSB7XG5cdFx0XHR0aHJvdyAobmV3IEVycm9yKGBbJHt0aXRsZX1dIFN0YXRlLm1lYW4gJHttZWFufSB3aXRoIGRpbWVuc2lvbiAke21lYW5EaW1lbnNpb259IGRvZXMgbm90IG1hdGNoIGV4cGVjdGVkIGRpbWVuc2lvbiAoJHtkaW1lbnNpb259KWApKTtcblx0XHR9XG5cblx0XHRjaGVja01hdHJpeChtZWFuLCBbbWVhbkRpbWVuc2lvbiwgMV0sIHRpdGxlID8gdGl0bGUgKyAnLW1lYW4nIDogJ21lYW4nKTtcblx0XHRjaGVja01hdHJpeChjb3ZhcmlhbmNlLCBbbWVhbkRpbWVuc2lvbiwgbWVhbkRpbWVuc2lvbl0sIHRpdGxlID8gdGl0bGUgKyAnLWNvdmFyaWFuY2UnIDogJ2NvdmFyaWFuY2UnKTtcblx0XHRjaGVja0NvdmFyaWFuY2Uoe2NvdmFyaWFuY2UsIGVpZ2VufSwgdGl0bGUgPyB0aXRsZSArICctY292YXJpYW5jZScgOiAnY292YXJpYW5jZScpO1xuXHRcdC8vIElmICh0eXBlb2YgKGluZGV4KSAhPT0gJ251bWJlcicpIHtcblx0XHQvLyBcdHRocm93IChuZXcgVHlwZUVycm9yKCd0IG11c3QgYmUgYSBudW1iZXInKSk7XG5cdFx0Ly8gfVxuXHR9XG5cblx0c3RhdGljIG1hdE11bCh7c3RhdGUsIG1hdHJpeH0pIHtcblx0XHRjb25zdCBjb3ZhcmlhbmNlID0gbWF0TXVsKFxuXHRcdFx0bWF0TXVsKG1hdHJpeCwgc3RhdGUuY292YXJpYW5jZSksXG5cdFx0XHR0cmFuc3Bvc2UobWF0cml4KVxuXHRcdCk7XG5cdFx0Y29uc3QgbWVhbiA9IG1hdE11bChtYXRyaXgsIHN0YXRlLm1lYW4pO1xuXG5cdFx0cmV0dXJuIG5ldyBTdGF0ZSh7XG5cdFx0XHRtZWFuLFxuXHRcdFx0Y292YXJpYW5jZSxcblx0XHRcdGluZGV4OiBzdGF0ZS5pbmRleFxuXHRcdH0pO1xuXHR9XG5cblx0LyoqXG5cdCogRnJvbSBhIHN0YXRlIGluIG4tZGltZW5zaW9uIGNyZWF0ZSBhIHN0YXRlIGluIGEgc3Vic3BhY2Vcblx0KiBJZiB5b3Ugc2VlIHRoZSBzdGF0ZSBhcyBhIE4tZGltZW5zaW9uIGdhdXNzaWFuLFxuXHQqIHRoaXMgY2FuIGJlIHZpZXdlZCBhcyB0aGUgc3ViIE0tZGltZW5zaW9uIGdhdXNzaWFuIChNIDwgTilcblx0KiBAcGFyYW0ge0FycmF5LjxOdW1iZXI+fSBvYnNJbmRleGVzIGxpc3Qgb2YgZGltZW5zaW9uIHRvIGV4dHJhY3QsICAoTSA8IE4gPD0+IG9ic0luZGV4ZXMubGVuZ3RoIDwgdGhpcy5tZWFuLmxlbmd0aClcblx0KiBAcmV0dXJucyB7U3RhdGV9IHN1YlN0YXRlIGluIHN1YnNwYWNlLCB3aXRoIHN1YlN0YXRlLm1lYW4ubGVuZ3RoID09PSBvYnNJbmRleGVzLmxlbmd0aFxuXHQqL1xuXHRzdWJTdGF0ZShvYnNJbmRleGVzKSB7XG5cdFx0Y29uc3Qgc3RhdGUgPSBuZXcgU3RhdGUoe1xuXHRcdFx0bWVhbjogb2JzSW5kZXhlcy5tYXAoaSA9PiB0aGlzLm1lYW5baV0pLFxuXHRcdFx0Y292YXJpYW5jZTogc3ViU3F1YXJlTWF0cml4KHRoaXMuY292YXJpYW5jZSwgb2JzSW5kZXhlcyksXG5cdFx0XHRpbmRleDogdGhpcy5pbmRleFxuXHRcdH0pO1xuXHRcdHJldHVybiBzdGF0ZTtcblx0fVxuXG5cdC8qKlxuXHQqIFNpbXBsZSBNYWxhaGFub2JpcyBkaXN0YW5jZSBiZXR3ZWVuIHRoZSBkaXN0cmlidXRpb24gKHRoaXMpIGFuZCBhIHBvaW50XG5cdCogQHBhcmFtIHtBcnJheS48W051bWJlcl0+fSBwb2ludCBhIE54MSBtYXRyaXggcmVwcmVzZW50aW5nIGEgcG9pbnRcblx0Ki9cblx0cmF3RGV0YWlsZWRNYWhhbGFub2Jpcyhwb2ludCkge1xuXHRcdGNvbnN0IGRpZmYgPSBzdWIodGhpcy5tZWFuLCBwb2ludCk7XG5cdFx0dGhpcy5jaGVjaygpO1xuXHRcdGNvbnN0IGNvdmFyaWFuY2VJbnZlcnQgPSBpbnZlcnQodGhpcy5jb3ZhcmlhbmNlKTtcblx0XHRpZiAoY292YXJpYW5jZUludmVydCA9PT0gbnVsbCkge1xuXHRcdFx0dGhpcy5jaGVjayh7ZWlnZW46IHRydWV9KTtcblx0XHRcdHRocm93IChuZXcgRXJyb3IoYENhbm5vdCBpbnZlcnQgY292YXJpYW5jZSAke0pTT04uc3RyaW5naWZ5KHRoaXMuY292YXJpYW5jZSl9YCkpO1xuXHRcdH1cblxuXHRcdGNvbnN0IGRpZmZUcmFuc3Bvc2VkID0gdHJhbnNwb3NlKGRpZmYpO1xuXG5cdFx0Ly8gQ29uc29sZS5sb2coJ2NvdmFyaWFuY2UgaW4gb2JzIHNwYWNlJywgY292YXJpYW5jZUluT2JzZXJ2YXRpb25TcGFjZSk7XG5cblx0XHRjb25zdCB2YWx1ZSA9IE1hdGguc3FydChcblx0XHRcdG1hdE11bChcblx0XHRcdFx0bWF0TXVsKFxuXHRcdFx0XHRcdGRpZmZUcmFuc3Bvc2VkLFxuXHRcdFx0XHRcdGNvdmFyaWFuY2VJbnZlcnRcblx0XHRcdFx0KSxcblx0XHRcdFx0ZGlmZlxuXHRcdFx0KVxuXHRcdCk7XG5cdFx0aWYgKE51bWJlci5pc05hTih2YWx1ZSkpIHtcblx0XHRcdGNvbnNvbGUubG9nKHtkaWZmLCBjb3ZhcmlhbmNlSW52ZXJ0LCB0aGlzOiB0aGlzLCBwb2ludH0sIG1hdE11bChcblx0XHRcdFx0bWF0TXVsKFxuXHRcdFx0XHRcdGRpZmZUcmFuc3Bvc2VkLFxuXHRcdFx0XHRcdGNvdmFyaWFuY2VJbnZlcnRcblx0XHRcdFx0KSxcblx0XHRcdFx0ZGlmZlxuXHRcdFx0KSk7XG5cdFx0XHR0aHJvdyAobmV3IEVycm9yKCdtYWhhbGFub2JpcyBpcyBOYU4nKSk7XG5cdFx0fVxuXG5cdFx0cmV0dXJuIHtcblx0XHRcdGRpZmYsXG5cdFx0XHRjb3ZhcmlhbmNlSW52ZXJ0LFxuXHRcdFx0dmFsdWVcblx0XHR9O1xuXHR9XG5cblx0LyoqXG5cdCogTWFsYWhhbm9iaXMgZGlzdGFuY2UgaXMgbWFkZSBhZ2FpbnN0IGFuIG9ic2VydmF0aW9uLCBzbyB0aGUgbWVhbiBhbmQgY292YXJpYW5jZVxuXHQqIGFyZSBwcm9qZWN0ZWQgaW50byB0aGUgb2JzZXJ2YXRpb24gc3BhY2Vcblx0KiBAcGFyYW0ge0thbG1hbkZpbHRlcn0ga2Yga2FsbWFuIGZpbHRlciB1c2UgdG8gcHJvamVjdCB0aGUgc3RhdGUgaW4gb2JzZXJ2YXRpb24ncyBzcGFjZVxuXHQqIEBwYXJhbSB7T2JzZXJ2YXRpb259IG9ic2VydmF0aW9uXG5cdCogQHBhcmFtIHtBcnJheS48TnVtYmVyPn0gb2JzSW5kZXhlcyBsaXN0IG9mIGluZGV4ZXMgb2Ygb2JzZXJ2YXRpb24gc3RhdGUgdG8gdXNlIGZvciB0aGUgbWFoYWxhbm9iaXMgZGlzdGFuY2Vcblx0Ki9cblx0ZGV0YWlsZWRNYWhhbGFub2Jpcyh7a2YsIG9ic2VydmF0aW9uLCBvYnNJbmRleGVzfSkge1xuXHRcdGlmIChvYnNlcnZhdGlvbi5sZW5ndGggIT09IGtmLm9ic2VydmF0aW9uLmRpbWVuc2lvbikge1xuXHRcdFx0dGhyb3cgKG5ldyBFcnJvcihgTWFoYWxhbm9iaXMgb2JzZXJ2YXRpb24gJHtvYnNlcnZhdGlvbn0gKGRpbWVuc2lvbjogJHtvYnNlcnZhdGlvbi5sZW5ndGh9KSBkb2VzIG5vdCBtYXRjaCB3aXRoIGtmIG9ic2VydmF0aW9uIGRpbWVuc2lvbiAoJHtrZi5vYnNlcnZhdGlvbi5kaW1lbnNpb259KWApKTtcblx0XHR9XG5cblx0XHRsZXQgY29ycmVjdGx5U2l6ZWRPYnNlcnZhdGlvbiA9IGFycmF5VG9NYXRyaXgoe29ic2VydmF0aW9uLCBkaW1lbnNpb246IG9ic2VydmF0aW9uLmxlbmd0aH0pO1xuXG5cdFx0Y29uc3Qgc3RhdGVQcm9qZWN0aW9uID0ga2YuZ2V0VmFsdWUoa2Yub2JzZXJ2YXRpb24uc3RhdGVQcm9qZWN0aW9uLCB7fSk7XG5cblx0XHRsZXQgcHJvamVjdGVkU3RhdGUgPSB0aGlzLmNvbnN0cnVjdG9yLm1hdE11bCh7c3RhdGU6IHRoaXMsIG1hdHJpeDogc3RhdGVQcm9qZWN0aW9ufSk7XG5cblx0XHRpZiAoQXJyYXkuaXNBcnJheShvYnNJbmRleGVzKSkge1xuXHRcdFx0cHJvamVjdGVkU3RhdGUgPSBwcm9qZWN0ZWRTdGF0ZS5zdWJTdGF0ZShvYnNJbmRleGVzKTtcblx0XHRcdGNvcnJlY3RseVNpemVkT2JzZXJ2YXRpb24gPSBvYnNJbmRleGVzLm1hcChpID0+IGNvcnJlY3RseVNpemVkT2JzZXJ2YXRpb25baV0pO1xuXHRcdH1cblxuXHRcdHJldHVybiBwcm9qZWN0ZWRTdGF0ZS5yYXdEZXRhaWxlZE1haGFsYW5vYmlzKGNvcnJlY3RseVNpemVkT2JzZXJ2YXRpb24pO1xuXHR9XG5cblx0LyoqXG5cdCogQHBhcmFtIHtLYWxtYW5GaWx0ZXJ9IGtmIGthbG1hbiBmaWx0ZXIgdXNlIHRvIHByb2plY3QgdGhlIHN0YXRlIGluIG9ic2VydmF0aW9uJ3Mgc3BhY2Vcblx0KiBAcGFyYW0ge09ic2VydmF0aW9ufSBvYnNlcnZhdGlvblxuXHQqIEBwYXJhbSB7QXJyYXkuPE51bWJlcj59IG9ic0luZGV4ZXMgbGlzdCBvZiBpbmRleGVzIG9mIG9ic2VydmF0aW9uIHN0YXRlIHRvIHVzZSBmb3IgdGhlIG1haGFsYW5vYmlzIGRpc3RhbmNlXG5cdCogQHJldHVybnMge051bWJlcn1cblx0Ki9cblx0bWFoYWxhbm9iaXMob3B0aW9ucykge1xuXHRcdGNvbnN0IHJlc3VsdCA9IHRoaXMuZGV0YWlsZWRNYWhhbGFub2JpcyhvcHRpb25zKS52YWx1ZTtcblx0XHRpZiAoTnVtYmVyLmlzTmFOKHJlc3VsdCkpIHtcblx0XHRcdHRocm93IChuZXcgVHlwZUVycm9yKCdtYWhhbGFub2JpcyBpcyBOYU4nKSk7XG5cdFx0fVxuXG5cdFx0cmV0dXJuIHJlc3VsdDtcblx0fVxuXG5cdC8qKlxuXHQqIEJoYXR0YWNoYXJ5eWEgZGlzdGFuY2UgaXMgbWFkZSBhZ2FpbnN0IGluIHRoZSBvYnNlcnZhdGlvbiBzcGFjZVxuXHQqIHRvIGRvIGl0IGluIHRoZSBub3JtYWwgc3BhY2Ugc2VlIHN0YXRlLmJoYXR0YWNoYXJ5eWFcblx0KiBAcGFyYW0ge0thbG1hbkZpbHRlcn0ga2Yga2FsbWFuIGZpbHRlciB1c2UgdG8gcHJvamVjdCB0aGUgc3RhdGUgaW4gb2JzZXJ2YXRpb24ncyBzcGFjZVxuXHQqIEBwYXJhbSB7U3RhdGV9IHN0YXRlXG5cdCogQHBhcmFtIHtBcnJheS48TnVtYmVyPn0gb2JzSW5kZXhlcyBsaXN0IG9mIGluZGV4ZXMgb2Ygb2JzZXJ2YXRpb24gc3RhdGUgdG8gdXNlIGZvciB0aGUgYmhhdHRhY2hhcnl5YSBkaXN0YW5jZVxuXHQqIEByZXR1cm5zIHtOdW1iZXJ9XG5cdCovXG5cdG9ic0JoYXR0YWNoYXJ5eWEoe2tmLCBzdGF0ZSwgb2JzSW5kZXhlc30pIHtcblx0XHRjb25zdCBzdGF0ZVByb2plY3Rpb24gPSBrZi5nZXRWYWx1ZShrZi5vYnNlcnZhdGlvbi5zdGF0ZVByb2plY3Rpb24sIHt9KTtcblxuXHRcdGxldCBwcm9qZWN0ZWRTZWxmU3RhdGUgPSB0aGlzLmNvbnN0cnVjdG9yLm1hdE11bCh7c3RhdGU6IHRoaXMsIG1hdHJpeDogc3RhdGVQcm9qZWN0aW9ufSk7XG5cdFx0bGV0IHByb2plY3RlZE90aGVyU3RhdGUgPSB0aGlzLmNvbnN0cnVjdG9yLm1hdE11bCh7c3RhdGUsIG1hdHJpeDogc3RhdGVQcm9qZWN0aW9ufSk7XG5cblx0XHRpZiAoQXJyYXkuaXNBcnJheShvYnNJbmRleGVzKSkge1xuXHRcdFx0cHJvamVjdGVkU2VsZlN0YXRlID0gcHJvamVjdGVkU2VsZlN0YXRlLnN1YlN0YXRlKG9ic0luZGV4ZXMpO1xuXHRcdFx0cHJvamVjdGVkT3RoZXJTdGF0ZSA9IHByb2plY3RlZE90aGVyU3RhdGUuc3ViU3RhdGUob2JzSW5kZXhlcyk7XG5cdFx0fVxuXG5cdFx0cmV0dXJuIHByb2plY3RlZFNlbGZTdGF0ZS5iaGF0dGFjaGFyeXlhKHByb2plY3RlZE90aGVyU3RhdGUpO1xuXHR9XG5cblx0LyoqXG5cdCogQHBhcmFtIHtTdGF0ZX0gb3RoZXJTdGF0ZSBvdGhlciBzdGF0ZSB0byBjb21wYXJlIHdpdGhcblx0KiBAcmV0dXJucyB7TnVtYmVyfVxuXHQqL1xuXHRiaGF0dGFjaGFyeXlhKG90aGVyU3RhdGUpIHtcblx0XHRjb25zdCBzdGF0ZSA9IHRoaXM7XG5cdFx0Y29uc3QgYXZlcmFnZSA9IGVsZW1XaXNlKFtzdGF0ZS5jb3ZhcmlhbmNlLCBvdGhlclN0YXRlLmNvdmFyaWFuY2VdLCAoW2EsIGJdKSA9PiAoYSArIGIpIC8gMik7XG5cblx0XHRsZXQgY292YXJJbnZlcnRlZDtcblx0XHR0cnkge1xuXHRcdFx0Y292YXJJbnZlcnRlZCA9IGludmVydChhdmVyYWdlKTtcblx0XHR9IGNhdGNoIChlcnJvcikge1xuXHRcdFx0Y29uc29sZS5sb2coJ0Nhbm5vdCBpbnZlcnQnLCBhdmVyYWdlKTtcblx0XHRcdHRocm93IChlcnJvcik7XG5cdFx0fVxuXG5cdFx0Y29uc3QgZGlmZiA9IHN1YihzdGF0ZS5tZWFuLCBvdGhlclN0YXRlLm1lYW4pO1xuXG5cdFx0cmV0dXJuIG1hdE11bCh0cmFuc3Bvc2UoZGlmZiksIG1hdE11bChjb3ZhckludmVydGVkLCBkaWZmKSlbMF1bMF07XG5cdH1cbn1cblxubW9kdWxlLmV4cG9ydHMgPSBTdGF0ZTtcbiIsIi8qKlxuKlJldHVybnMgdGhlIGNvcnJlc3BvbmRpbmcgbWF0cml4IGluIGRpbSoxLCBnaXZlbiBhbiBkaW0gbWF0cml4LCBhbmQgY2hlY2tzXG4qIGlmIGNvcnJlc3BvbmRpbmcgd2l0aCB0aGUgb2JzZXJ2YXRpb24gZGltZW5zaW9uXG4qQHBhcmFtIHtBcnJheS48TnVtYmVyPiB8IEFycmF5LjxBcnJheS48TnVtYmVyPj59IG9ic2VydmF0aW9uXG4qQHBhcmFtIHtOdW1iZXJ9IGRpbWVuc2lvblxuKkByZXR1cm5zIHtBcnJheS48QXJyYXkuPE51bWJlcj4+fVxuKi9cblxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiAoe29ic2VydmF0aW9uLCBkaW1lbnNpb259KSB7XG5cdGlmICghQXJyYXkuaXNBcnJheShvYnNlcnZhdGlvbikpIHtcblx0XHRpZiAoZGltZW5zaW9uID09PSAxICYmIHR5cGVvZiAob2JzZXJ2YXRpb24pID09PSAnbnVtYmVyJykge1xuXHRcdFx0cmV0dXJuIFtbb2JzZXJ2YXRpb25dXTtcblx0XHR9XG5cblx0XHR0aHJvdyAobmV3IFR5cGVFcnJvcihgVGhlIG9ic2VydmF0aW9uICgke29ic2VydmF0aW9ufSkgc2hvdWxkIGJlIGFuIGFycmF5IChkaW1lbnNpb246ICR7ZGltZW5zaW9ufSlgKSk7XG5cdH1cblxuXHRpZiAob2JzZXJ2YXRpb24ubGVuZ3RoICE9PSBkaW1lbnNpb24pIHtcblx0XHR0aHJvdyAobmV3IFR5cGVFcnJvcihgT2JzZXJ2YXRpb24gKCR7b2JzZXJ2YXRpb24ubGVuZ3RofSkgYW5kIGRpbWVuc2lvbiAoJHtkaW1lbnNpb259KSBub3QgbWF0Y2hpbmdgKSk7XG5cdH1cblxuXHRpZiAodHlwZW9mIChvYnNlcnZhdGlvblswXSkgPT09ICdudW1iZXInIHx8IG9ic2VydmF0aW9uWzBdID09PSBudWxsKSB7XG5cdFx0cmV0dXJuIG9ic2VydmF0aW9uLm1hcChlbGVtZW50ID0+IFtlbGVtZW50XSk7XG5cdH1cblxuXHRyZXR1cm4gb2JzZXJ2YXRpb247XG59O1xuIiwiY29uc3QgdG9sZXJhbmNlID0gMC4xO1xuY29uc3QgTWF0cml4ID0gcmVxdWlyZSgnQHJheXlhbWhrL21hdHJpeCcpO1xuY29uc3QgY2hlY2tNYXRyaXggPSByZXF1aXJlKCcuL2NoZWNrLW1hdHJpeCcpO1xuXG5jb25zdCBjaGVja0RlZmluaXRlUG9zaXRpdmUgPSBmdW5jdGlvbiAoY292YXJpYW5jZSwgdG9sZXJhbmNlID0gMWUtMTApIHtcblx0Y29uc3QgY292YXJpYW5jZU1hdHJpeCA9IG5ldyBNYXRyaXgoY292YXJpYW5jZSk7XG5cdGNvbnN0IGVpZ2VudmFsdWVzID0gY292YXJpYW5jZU1hdHJpeC5laWdlbnZhbHVlcygpO1xuXHRlaWdlbnZhbHVlcy5mb3JFYWNoKGVpZ2VudmFsdWUgPT4ge1xuXHRcdGlmIChlaWdlbnZhbHVlIDw9IC10b2xlcmFuY2UpIHtcblx0XHRcdGNvbnNvbGUubG9nKGNvdmFyaWFuY2UsIGVpZ2VudmFsdWUpO1xuXHRcdFx0dGhyb3cgbmV3IEVycm9yKGBFaWdlbnZhbHVlIHNob3VsZCBiZSBwb3NpdGl2ZSAoYWN0dWFsOiAke2VpZ2VudmFsdWV9KWApO1xuXHRcdH1cblx0fSk7XG5cdGNvbnNvbGUubG9nKCdpcyBkZWZpbml0ZSBwb3NpdGl2ZScsIGNvdmFyaWFuY2UpO1xufTtcblxuY29uc3QgY2hlY2tTeW1ldHJpYyA9IGZ1bmN0aW9uIChjb3ZhcmlhbmNlLCB0aXRsZSA9ICdjaGVja1N5bWV0cmljJykge1xuXHRjb3ZhcmlhbmNlLmZvckVhY2goKHJvdywgcm93SWQpID0+IHJvdy5mb3JFYWNoKChpdGVtLCBjb2xJZCkgPT4ge1xuXHRcdGlmIChyb3dJZCA9PT0gY29sSWQgJiYgaXRlbSA8IDApIHtcblx0XHRcdHRocm93IG5ldyBFcnJvcihgWyR7dGl0bGV9XSBWYXJpYW5jZVske2NvbElkfV0gc2hvdWxkIGJlIHBvc2l0aXZlIChhY3R1YWw6ICR7aXRlbX0pYCk7XG5cdFx0fSBlbHNlIGlmIChNYXRoLmFicyhpdGVtKSA+IE1hdGguc3FydChjb3ZhcmlhbmNlW3Jvd0lkXVtyb3dJZF0gKiBjb3ZhcmlhbmNlW2NvbElkXVtjb2xJZF0pKSB7XG5cdFx0XHRjb25zb2xlLmxvZyhjb3ZhcmlhbmNlKTtcblx0XHRcdHRocm93IG5ldyBFcnJvcihgWyR7dGl0bGV9XSBDb3ZhcmlhbmNlWyR7cm93SWR9XVske2NvbElkfV0gc2hvdWxkIHZlcmlmeSBDYXVjaHkgU2Nod2FyeiBJbmVxdWFsaXR5IGAgK1xuXHRcdFx0XHRgKGV4cGVjdGVkOiB8eHwgPD0gc3FydCgke2NvdmFyaWFuY2Vbcm93SWRdW3Jvd0lkXX0gKiAke2NvdmFyaWFuY2VbY29sSWRdW2NvbElkXX0pYCArXG5cdFx0XHRcdGAgYWN0dWFsOiAke2l0ZW19KWApO1xuXHRcdH0gZWxzZSBpZiAoTWF0aC5hYnMoaXRlbSAtIGNvdmFyaWFuY2VbY29sSWRdW3Jvd0lkXSkgPiB0b2xlcmFuY2UpIHtcblx0XHRcdHRocm93IG5ldyBFcnJvcihgWyR7dGl0bGV9XSBDb3ZhcmlhbmNlWyR7cm93SWR9XVske2NvbElkfV0gc2hvdWxkIGVxdWFsIENvdmFyaWFuY2VbJHtjb2xJZH1dWyR7cm93SWR9XSBgICtcblx0XHRcdGAgKGFjdHVhbCBkaWZmOiAke01hdGguYWJzKGl0ZW0gLSBjb3ZhcmlhbmNlW2NvbElkXVtyb3dJZF0pfSkgID0gJHtpdGVtfSAtICR7Y292YXJpYW5jZVtjb2xJZF1bcm93SWRdfVxcbmAgK1xuXHRcdFx0YCR7Y292YXJpYW5jZS5qb2luKCdcXG4nKX0gaXMgaW52YWxpZGBcblx0XHRcdCk7XG5cdFx0fVxuXHR9KSk7XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uICh7Y292YXJpYW5jZSwgZWlnZW4gPSBmYWxzZX0pIHtcblx0Y2hlY2tNYXRyaXgoY292YXJpYW5jZSk7XG5cdGNoZWNrU3ltZXRyaWMoY292YXJpYW5jZSk7XG5cdGlmIChlaWdlbikge1xuXHRcdGNoZWNrRGVmaW5pdGVQb3NpdGl2ZShjb3ZhcmlhbmNlKTtcblx0fVxufTtcbiIsImNvbnN0IGNoZWNrU2hhcGUgPSByZXF1aXJlKCcuL2NoZWNrLXNoYXBlJyk7XG5cbm1vZHVsZS5leHBvcnRzID0gZnVuY3Rpb24gKG1hdHJpeCwgc2hhcGUsIHRpdGxlID0gJ2NoZWNrTWF0cml4Jykge1xuXHRpZiAobWF0cml4LnJlZHVjZSgoYSwgYikgPT4gYS5jb25jYXQoYikpLmZpbHRlcihhID0+IE51bWJlci5pc05hTihhKSkubGVuZ3RoID4gMCkge1xuXHRcdHRocm93IChuZXcgRXJyb3IoXG5cdFx0XHRgWyR7dGl0bGV9XSBNYXRyaXggc2hvdWxkIG5vdCBoYXZlIGEgTmFOXFxuSW4gOiBcXG5gICtcblx0XHRcdG1hdHJpeC5qb2luKCdcXG4nKVxuXHRcdCkpO1xuXHR9XG5cblx0aWYgKHNoYXBlKSB7XG5cdFx0Y2hlY2tTaGFwZShtYXRyaXgsIHNoYXBlLCB0aXRsZSk7XG5cdH1cbn07XG4iLCJjb25zdCBjaGVja1NoYXBlID0gZnVuY3Rpb24gKG1hdHJpeCwgc2hhcGUsIHRpdGxlID0gJ2NoZWNrU2hhcGUnKSB7XG5cdGlmIChtYXRyaXgubGVuZ3RoICE9PSBzaGFwZVswXSkge1xuXHRcdHRocm93IChuZXcgRXJyb3IoYFske3RpdGxlfV0gZXhwZWN0ZWQgc2l6ZSAoJHtzaGFwZVswXX0pIGFuZCBsZW5ndGggKCR7bWF0cml4Lmxlbmd0aH0pIGRvZXMgbm90IG1hdGNoYCkpO1xuXHR9XG5cblx0aWYgKHNoYXBlLmxlbmd0aCA+IDEpIHtcblx0XHRyZXR1cm4gbWF0cml4LmZvckVhY2gobSA9PiBjaGVja1NoYXBlKG0sIHNoYXBlLnNsaWNlKDEpLCB0aXRsZSkpO1xuXHR9XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IGNoZWNrU2hhcGU7XG4iLCJjb25zdCBjaGVja0NvdmFyaWFuY2UgPSByZXF1aXJlKCcuL2NoZWNrLWNvdmFyaWFuY2UnKTtcblxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiAoe2NvcnJlbGF0aW9uLCB2YXJpYW5jZX0pIHtcblx0Y2hlY2tDb3ZhcmlhbmNlKHtjb3ZhcmlhbmNlOiBjb3JyZWxhdGlvbn0pO1xuXHRyZXR1cm4gY29ycmVsYXRpb24ubWFwKChjLCByb3dJbmRleCkgPT4gYy5tYXAoKGEsIGNvbEluZGV4KSA9PiBhICogTWF0aC5zcXJ0KHZhcmlhbmNlW2NvbEluZGV4XSAqIHZhcmlhbmNlW3Jvd0luZGV4XSkpKTtcbn07XG4iLCJjb25zdCBjaGVja0NvdmFyaWFuY2UgPSByZXF1aXJlKCcuL2NoZWNrLWNvdmFyaWFuY2UnKTtcblxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiAoY292YXJpYW5jZSkge1xuXHRjaGVja0NvdmFyaWFuY2Uoe2NvdmFyaWFuY2V9KTtcblx0Y29uc3QgdmFyaWFuY2UgPSBjb3ZhcmlhbmNlLm1hcCgoXywgaSkgPT4gY292YXJpYW5jZVtpXVtpXSk7XG5cblx0cmV0dXJuIHtcblx0XHR2YXJpYW5jZSxcblx0XHRjb3JyZWxhdGlvbjogY292YXJpYW5jZS5tYXAoKGMsIHJvd0luZGV4KSA9PiBjLm1hcCgoYSwgY29sSW5kZXgpID0+IGEgLyBNYXRoLnNxcnQodmFyaWFuY2VbY29sSW5kZXhdICogdmFyaWFuY2Vbcm93SW5kZXhdKSkpXG5cdH07XG59O1xuIiwiY29uc3QgdW5pcSA9IHJlcXVpcmUoJy4vdW5pcS5qcycpO1xuXG5jb25zdCBsaW1pdCA9IDEwMDtcblxuLyoqXG4qRXF1aXZhbGVudCB0byB0aGUgT2JqZWN0LmFzc2lnbiBtZXRob2RlLCB0YWtlcyBzZXZlcmFsIGFyZ3VtZW50cyBhbmQgY3JlYXRlcyBhIG5ldyBvYmplY3QgY29ycmVzcG9uZGluZyB0byB0aGUgYXNzaWdubWVudCBvZiB0aGUgYXJndW1lbnRzXG4qIEBwYXJhbSB7T2JqZWN0fSBhcmdzXG4qIEBwYXJhbSB7TnVtYmVyfSBzdGVwXG4qL1xuY29uc3QgZGVlcEFzc2lnbiA9IGZ1bmN0aW9uIChhcmdzLCBzdGVwKSB7XG5cdGlmIChzdGVwID4gbGltaXQpIHtcblx0XHR0aHJvdyAobmV3IEVycm9yKGBJbiBkZWVwQXNzaWduLCBudW1iZXIgb2YgcmVjdXJzaXZlIGNhbGwgKCR7c3RlcH0pIHJlYWNoZWQgbGltaXQgKCR7bGltaXR9KSwgZGVlcEFzc2lnbiBpcyBub3Qgd29ya2luZyBvbiAgc2VsZi1yZWZlcmVuY2luZyBvYmplY3RzYCkpO1xuXHR9XG5cblx0Y29uc3QgZmlsdGVyQXJndW1lbnRzID0gYXJncy5maWx0ZXIoYXJnID0+IHR5cGVvZiAoYXJnKSAhPT0gJ3VuZGVmaW5lZCcgJiYgYXJnICE9PSBudWxsKTtcblx0Y29uc3QgbGFzdEFyZ3VtZW50ID0gZmlsdGVyQXJndW1lbnRzW2ZpbHRlckFyZ3VtZW50cy5sZW5ndGggLSAxXTtcblx0aWYgKGZpbHRlckFyZ3VtZW50cy5sZW5ndGggPT09IDEpIHtcblx0XHRyZXR1cm4gZmlsdGVyQXJndW1lbnRzWzBdO1xuXHR9XG5cblx0aWYgKHR5cGVvZiAobGFzdEFyZ3VtZW50KSAhPT0gJ29iamVjdCcgfHwgQXJyYXkuaXNBcnJheShsYXN0QXJndW1lbnQpKSB7XG5cdFx0cmV0dXJuIGxhc3RBcmd1bWVudDtcblx0fVxuXG5cdGlmIChmaWx0ZXJBcmd1bWVudHMubGVuZ3RoID09PSAwKSB7XG5cdFx0cmV0dXJuIG51bGw7XG5cdH1cblxuXHRjb25zdCBvYmplY3RzQXJndW1lbnRzID0gZmlsdGVyQXJndW1lbnRzLmZpbHRlcihhcmcgPT4gdHlwZW9mIChhcmcpID09PSAnb2JqZWN0Jyk7XG5cdGxldCBrZXlzID0gW107XG5cdG9iamVjdHNBcmd1bWVudHMuZm9yRWFjaChhcmcgPT4ge1xuXHRcdGtleXMgPSBrZXlzLmNvbmNhdChPYmplY3Qua2V5cyhhcmcpKTtcblx0fSk7XG5cdGNvbnN0IHVuaXFLZXlzID0gdW5pcShrZXlzKTtcblx0Y29uc3QgcmVzdWx0ID0ge307XG5cdHVuaXFLZXlzLmZvckVhY2goa2V5ID0+IHtcblx0XHRjb25zdCB2YWx1ZXMgPSBvYmplY3RzQXJndW1lbnRzLm1hcChhcmcgPT4gYXJnW2tleV0pO1xuXHRcdHJlc3VsdFtrZXldID0gZGVlcEFzc2lnbih2YWx1ZXMsIHN0ZXAgKyAxKTtcblx0fSk7XG5cdHJldHVybiByZXN1bHQ7XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9ICgoLi4uYXJncykgPT4gZGVlcEFzc2lnbihhcmdzLCAwKSk7XG4iLCIvKipcbiogQHBhcmFtIHtPYmplY3R9IG9wdHNcbiogQHBhcmFtIHtBcnJheS48QXJyYXkuPE51bWJlcj4+fSBvcHRzLm1lYXN1cmVzIGEgbGlzdCBvZiBtZWFzdXJlLCBzaXplIGlzIEx4TiBMIHRoZSBudW1iZXIgb2Ygc2FtcGxlLCBOIHRoZSBkaW1lbnNpb25cbiogQHBhcmFtIHtBcnJheS48QXJyYXkuPE51bWJlcj4+fSBvcHRzLmF2ZXJhZ2VzIGEgbGlzdCBvZiBhdmVyYWdlcywgc2l6ZSBpcyBMeE4gTCB0aGUgbnVtYmVyIG9mIHNhbXBsZSwgTiB0aGUgZGltZW5zaW9uXG4qIEByZXR1cm5zIHtBcnJheS48QXJyYXkuPE51bWJlcj4+fSBjb3ZhcmlhbmNlIG1hdHJpeCBzaXplIGlzIE54TlxuKi9cblxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiAoe21lYXN1cmVzLCBhdmVyYWdlc30pIHtcblx0Y29uc3QgbCA9IG1lYXN1cmVzLmxlbmd0aDtcblx0Y29uc3QgbiA9IG1lYXN1cmVzWzBdLmxlbmd0aDtcblxuXHRpZiAobCA9PT0gMCkge1xuXHRcdHRocm93IChuZXcgRXJyb3IoJ0Nhbm5vdCBmaW5kIGNvdmFyaWFuY2UgZm9yIGVtcHR5IHNhbXBsZScpKTtcblx0fVxuXG5cdHJldHVybiAobmV3IEFycmF5KG4pLmZpbGwoMSkpLm1hcCgoXywgcm93SW5kZXgpID0+IHtcblx0XHRyZXR1cm4gKG5ldyBBcnJheShuKS5maWxsKDEpKS5tYXAoKF8sIGNvbEluZGV4KSA9PiB7XG5cdFx0XHRjb25zdCBzdGRzID0gbWVhc3VyZXMubWFwKChtLCBpKSA9PiAobVtyb3dJbmRleF0gLSBhdmVyYWdlc1tpXVtyb3dJbmRleF0pICogKG1bY29sSW5kZXhdIC0gYXZlcmFnZXNbaV1bY29sSW5kZXhdKSk7XG5cdFx0XHRjb25zdCByZXN1bHQgPSBzdGRzLnJlZHVjZSgoYSwgYikgPT4gYSArIGIpIC8gbDtcblx0XHRcdGlmIChOdW1iZXIuaXNOYU4ocmVzdWx0KSkge1xuXHRcdFx0XHR0aHJvdyAobmV3IFR5cGVFcnJvcigncmVzdWx0IGlzIE5hTicpKTtcblx0XHRcdH1cblxuXHRcdFx0cmV0dXJuIHJlc3VsdDtcblx0XHR9KTtcblx0fSk7XG59O1xuIiwiLyoqXG4qIEB0eXBlZGVmIHtOdW1iZXIgfCBBcnJheS48TnVtYmVyPiB8IEFycmF5LjxBcnJheS48TnVtYmVyPj59IENvdmFyaWFuY2VQYXJhbVxuKi9cbmNvbnN0IGRpYWcgPSByZXF1aXJlKCcuLi9saW5hbGdlYnJhL2RpYWcnKTtcbmNvbnN0IGNoZWNrTWF0cml4ID0gcmVxdWlyZSgnLi9jaGVjay1tYXRyaXgnKTtcbi8qKlxuKiBJZiBjb3YgaXMgYSBudW1iZXIsIHJlc3VsdCB3aWxsIGJlIElkZW50aXR5KmNvdlxuKiBJZiBjb3YgaXMgYW4gQXJyYXkuPE51bWJlcj4sIHJlc3VsdCB3aWxsIGJlIGRpYWcoY292KVxuKiBJZiBjb3YgaXMgYW4gQXJyYXkuPEFycmF5LjxOdW1iZXI+PiwgcmVzdWx0IHdpbGwgYmUgY292XG4qIEBwYXJhbSB7Q292YXJpYW5jZVBhcmFtfSBjb3ZcbiogQHBhcmFtIHtOdW1iZXJ9IGRpbWVuc2lvblxuKiBAcmV0dXJucyB7QXJyYXkuPEFycmF5LjxOdW1iZXI+Pn1cbiovXG5tb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uIChhcnJheSwge2RpbWVuc2lvbiwgdGl0bGUgPSAncG9seW1vcnBoJ30gPSB7fSkge1xuXHRpZiAodHlwZW9mIChhcnJheSkgPT09ICdudW1iZXInIHx8IEFycmF5LmlzQXJyYXkoYXJyYXkpKSB7XG5cdFx0aWYgKHR5cGVvZiAoYXJyYXkpID09PSAnbnVtYmVyJyAmJiB0eXBlb2YgKGRpbWVuc2lvbikgPT09ICdudW1iZXInKSB7XG5cdFx0XHRyZXR1cm4gZGlhZyhuZXcgQXJyYXkoZGltZW5zaW9uKS5maWxsKGFycmF5KSk7XG5cdFx0fVxuXG5cdFx0aWYgKChBcnJheS5pc0FycmF5KGFycmF5KSkgJiYgKEFycmF5LmlzQXJyYXkoYXJyYXlbMF0pKSkge1xuXHRcdFx0bGV0IHNoYXBlO1xuXHRcdFx0aWYgKHR5cGVvZiAoZGltZW5zaW9uKSA9PT0gJ251bWJlcicpIHtcblx0XHRcdFx0c2hhcGUgPSBbZGltZW5zaW9uLCBkaW1lbnNpb25dO1xuXHRcdFx0fVxuXG5cdFx0XHRjaGVja01hdHJpeChhcnJheSwgc2hhcGUsIHRpdGxlKTtcblx0XHRcdHJldHVybiBhcnJheTtcblx0XHR9XG5cblx0XHRpZiAoKEFycmF5LmlzQXJyYXkoYXJyYXkpKSAmJiAodHlwZW9mIChhcnJheVswXSkgPT09ICdudW1iZXInKSkge1xuXHRcdFx0cmV0dXJuIGRpYWcoYXJyYXkpO1xuXHRcdH1cblx0fVxuXG5cdHJldHVybiBhcnJheTtcbn07XG4iLCIvLyBDb25zdCBkaWFnID0gcmVxdWlyZSgnLi4vbGluYWxnZWJyYS9kaWFnLmpzJyk7XG5cbi8qKlxuKiBAY2FsbGJhY2sgTWF0cml4Q2FsbGJhY2tcbiogQHJldHVybnMgPEFycmF5LjxBcnJheS48TnVtYmVyPj5cbiovXG5cbi8qKlxuKiBUcmFuZm9ybXM6XG4qKiBhIDJkIGFycmF5IGludG8gYSBmdW5jdGlvbiAoKCkgPT4gYXJyYXkpXG4qKiBhIDFkIGFycmF5IGludG8gYSBmdW5jdGlvbiAoKCkgPT4gZGlhZyhhcnJheSkpXG4qQHBhcmFtIHtBcnJheS48TnVtYmVyPiB8IEFycmF5LjxBcnJheS48TnVtYmVyPj59IGFycmF5XG4qQHJldHVybnMge01hdHJpeENhbGxiYWNrfVxuKi9cblxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiAoYXJyYXkpIHtcblx0aWYgKHR5cGVvZiAoYXJyYXkpID09PSAnZnVuY3Rpb24nKSB7XG5cdFx0cmV0dXJuIGFycmF5O1xuXHR9XG5cblx0aWYgKEFycmF5LmlzQXJyYXkoYXJyYXkpKSB7XG5cdFx0cmV0dXJuIGFycmF5O1xuXHR9XG5cblx0dGhyb3cgKG5ldyBFcnJvcignT25seSBhcnJheXMgYW5kIGZ1bmN0aW9ucyBhcmUgYXV0aG9yaXplZCcpKTtcbn07XG4iLCJtb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uIChhcnJheSkge1xuXHRyZXR1cm4gYXJyYXkuZmlsdGVyKCh2YWx1ZSwgaW5kZXgpID0+XG5cdFx0YXJyYXkuaW5kZXhPZih2YWx1ZSkgPT09IGluZGV4XG5cdCk7XG59O1xuIiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbi8qKlxyXG4gKiBDYWxjdWxhdGVzIHRoZSBhcmd1bWVudCBvZiBhIENvbXBsZXggTnVtYmVyIHdoaWNoIGlzIHJlc3RyaWN0ZWQgdG8gdGhlIGludGVydmFsIFsgMCwgMs+AICkuPGJyPjxicj5cclxuICogXHJcbiAqIFRoZSBhcmd1bWVudCBvZiB0aGUgQ29tcGxleCBOdW1iZXIgaXMgdGhlIGFuZ2xlIGJldHdlZW4gcG9zaXRpdmUgcmVhbC1heGlzXHJcbiAqIGFuZCB0aGUgdmVjdG9yIHJlcHJlc2VudGluZyB0aGUgQ29tcGxleCBOdW1iZXIgb24gQ29tcGxleCBwbGFuZS48YnI+PGJyPlxyXG4gKiBcclxuICogSWYgdGhlIGdpdmVuIENvbXBsZXggTnVtYmVyIGlzIGNvbnNpZGVyZWQgYXMgMCwgcmV0dXJucyB1bmRlZmluZWQuXHJcbiAqIEBtZW1iZXJvZiBDb21wbGV4XHJcbiAqIEBpbnN0YW5jZVxyXG4gKiBAcmV0dXJucyB7bnVtYmVyfSBUaGUgYXJndW1lbnQgb2YgdGhlIENvbXBsZXggTnVtYmVyXHJcbiAqL1xuZnVuY3Rpb24gZ2V0QXJndW1lbnQoKSB7XG4gIHZhciB4ID0gdGhpcy5yZTtcbiAgdmFyIHkgPSB0aGlzLmltO1xuICB2YXIgZXBzaWxvbiA9IDEgLyAoTWF0aC5wb3coMTAsIDE1KSAqIDIpO1xuXG4gIGlmIChNYXRoLmFicyh4KSA8IGVwc2lsb24gJiYgTWF0aC5hYnMoeSkgPCBlcHNpbG9uKSB7XG4gICAgcmV0dXJuIHVuZGVmaW5lZDtcbiAgfVxuXG4gIGlmICh4ID09PSAwKSB7XG4gICAgaWYgKHkgPiAwKSB7XG4gICAgICByZXR1cm4gTWF0aC5QSSAqIDAuNTtcbiAgICB9XG5cbiAgICByZXR1cm4gTWF0aC5QSSAqIDEuNTtcbiAgfVxuXG4gIGlmICh5ID09PSAwKSB7XG4gICAgaWYgKHggPiAwKSB7XG4gICAgICByZXR1cm4gMDtcbiAgICB9XG5cbiAgICByZXR1cm4gTWF0aC5QSTtcbiAgfVxuXG4gIGlmICh4ID4gMCAmJiB5ID4gMCkge1xuICAgIHJldHVybiBNYXRoLmF0YW4oeSAvIHgpO1xuICB9XG5cbiAgaWYgKHggPCAwICYmIHkgPiAwKSB7XG4gICAgcmV0dXJuIE1hdGguUEkgLSBNYXRoLmF0YW4oeSAvICh4ICogLTEpKTtcbiAgfVxuXG4gIGlmICh4IDwgMCAmJiB5IDwgMCkge1xuICAgIHJldHVybiBNYXRoLlBJICsgTWF0aC5hdGFuKHkgKiAtMSAvICh4ICogLTEpKTtcbiAgfVxuXG4gIHJldHVybiBNYXRoLlBJICogMiAtIE1hdGguYXRhbih5ICogLTEgLyB4KTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBnZXRBcmd1bWVudDsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIEdldHMgdGhlIGltYWdpbmFyeSBwYXJ0IG9mIGEgQ29tcGxleCBOdW1iZXIuXHJcbiAqIEBtZW1iZXJvZiBDb21wbGV4XHJcbiAqIEBpbnN0YW5jZVxyXG4gKiBAcmV0dXJucyB7bnVtYmVyfSBUaGUgaW1hZ2luYXJ5IHBhcnQgb2YgdGhlIENvbXBsZXggTnVtYmVyXHJcbiAqL1xuZnVuY3Rpb24gZ2V0SW1hZ2luYXJ5KCkge1xuICByZXR1cm4gdGhpcy5pbTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBnZXRJbWFnaW5hcnk7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbi8qKlxyXG4gKiBDYWxjdWxhdGVzIHRoZSBtb2R1bHVzIG9mIGEgQ29tcGxleCBOdW1iZXIuPGJyPjxicj5cclxuICogXHJcbiAqIFRoZSBtb2R1bHVzIG9mIHRoZSBjb21wbGV4IG51bWJlciBpcyB0aGUgbGVuZ3RoIG9mIHRoZSB2ZWN0b3JcclxuICogcmVwcmVzZW50aW5nIHRoZSBjb21wbGV4IG51bWJlciBvbiBjb21wbGV4IHBsYW5lLlxyXG4gKiBAbWVtYmVyb2YgQ29tcGxleFxyXG4gKiBAaW5zdGFuY2VcclxuICogQHJldHVybnMge251bWJlcn0gVGhlIG1vZHVsdXMgb2YgdGhlIENvbXBsZXggTnVtYmVyXHJcbiAqL1xuZnVuY3Rpb24gZ2V0TW9kdWx1cygpIHtcbiAgcmV0dXJuIE1hdGguc3FydChNYXRoLnBvdyh0aGlzLnJlLCAyKSArIE1hdGgucG93KHRoaXMuaW0sIDIpKTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBnZXRNb2R1bHVzOyIsIlwidXNlIHN0cmljdFwiO1xuXG4vKipcclxuICogR2V0cyB0aGUgcmVhbCBwYXJ0IG9mIGEgQ29tcGxleCBOdW1iZXIuXHJcbiAqIEBtZW1iZXJvZiBDb21wbGV4XHJcbiAqIEBpbnN0YW5jZVxyXG4gKiBAcmV0dXJucyB7bnVtYmVyfSBUaGUgcmVhbCBwYXJ0IG9mIHRoZSBDb21wbGV4IE51bWJlclxyXG4gKi9cbmZ1bmN0aW9uIGdldFJlYWwoKSB7XG4gIHJldHVybiB0aGlzLnJlO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IGdldFJlYWw7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbi8qKlxyXG4gKiBHZXRzIHRoZSBzdHJpbmdpZmllZCBhbmQgZm9ybWF0dGVkIENvbXBsZXggTnVtYmVyLlxyXG4gKiBAbWVtYmVyb2YgQ29tcGxleFxyXG4gKiBAaW5zdGFuY2VcclxuICogQHJldHVybnMge3N0cmluZ30gVGhlIHN0cmluZ2lmaWVkIGFuZCBmb3JtYXR0ZWQgQ29tcGxleCBOdW1iZXJcclxuICovXG5mdW5jdGlvbiB0b1N0cmluZygpIHtcbiAgdmFyIHJlID0gdGhpcy5yZSxcbiAgICAgIGltID0gdGhpcy5pbTtcblxuICBpZiAoTnVtYmVyLmlzTmFOKHJlKSB8fCBOdW1iZXIuaXNOYU4oaW0pKSB7XG4gICAgcmV0dXJuICdOYU4nO1xuICB9XG5cbiAgaWYgKHJlID09PSAwICYmIGltID09PSAwKSB7XG4gICAgcmV0dXJuICcwJztcbiAgfVxuXG4gIGlmIChyZSA9PT0gMCkge1xuICAgIGlmIChpbSA9PT0gMSkge1xuICAgICAgcmV0dXJuICdpJztcbiAgICB9XG5cbiAgICBpZiAoaW0gPT09IC0xKSB7XG4gICAgICByZXR1cm4gJy1pJztcbiAgICB9XG5cbiAgICByZXR1cm4gXCJcIi5jb25jYXQoaW0sIFwiaVwiKTtcbiAgfVxuXG4gIGlmIChpbSA9PT0gMCkge1xuICAgIHJldHVybiBcIlwiLmNvbmNhdChyZSk7XG4gIH1cblxuICBpZiAoaW0gPiAwKSB7XG4gICAgaWYgKGltID09PSAxKSB7XG4gICAgICByZXR1cm4gXCJcIi5jb25jYXQocmUsIFwiICsgaVwiKTtcbiAgICB9XG5cbiAgICByZXR1cm4gXCJcIi5jb25jYXQocmUsIFwiICsgXCIpLmNvbmNhdChpbSwgXCJpXCIpO1xuICB9XG5cbiAgaWYgKGltID09PSAtMSkge1xuICAgIHJldHVybiBcIlwiLmNvbmNhdChyZSwgXCIgLSBpXCIpO1xuICB9XG5cbiAgcmV0dXJuIFwiXCIuY29uY2F0KHJlLCBcIiAtIFwiKS5jb25jYXQoTWF0aC5hYnMoaW0pLCBcImlcIik7XG59XG5cbm1vZHVsZS5leHBvcnRzID0gdG9TdHJpbmc7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbi8qKlxyXG4gKiBDYWxjdWxhdGVzIHRoZSBpbnZlcnNlIGNvc2luZSBvZiBhIENvbXBsZXggTnVtYmVyLlxyXG4gKiBAbWVtYmVyb2YgQ29tcGxleFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7Q29tcGxleH0gbnVtIC0gQW55IENvbXBsZXggTnVtYmVyXHJcbiAqIEByZXR1cm5zIHtDb21wbGV4fSBUaGUgcmVzdWx0IG9mIGludmVyc2UgY29zaW5lIGZ1bmN0aW9uXHJcbiAqL1xuZnVuY3Rpb24gYWNvcyhudW0pIHtcbiAgcmV0dXJuIHRoaXMuc3VidHJhY3QobmV3IHRoaXMoTWF0aC5QSSAvIDIpLCB0aGlzLmFzaW4obnVtKSk7XG59XG5cbm1vZHVsZS5leHBvcnRzID0gYWNvczsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIENhbGN1bGF0ZXMgdGhlIGludmVyc2UgY290YW5nZW50IG9mIGEgQ29tcGxleCBOdW1iZXIuXHJcbiAqIFRoZSBkb21haW4gb2YgdGhpcyBmdW5jdGlvbiBpcyBDIC8geyBpICwgLWkgLCAwIH0uPGJyPjxicj5cclxuICogXHJcbiAqIElmIHRoZSBhcmd1bWVudCBpcyBvdXQgb2YgaXRzIGRvbWFpbiwgaXQgcmV0dXJucyBDb21wbGV4Lk5hTi5cclxuICogQG1lbWJlcm9mIENvbXBsZXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge0NvbXBsZXh9IG51bSAtIEFueSBDb21wbGV4IE51bWJlciBleGNlcHQgaSwgLWkgYW5kIDBcclxuICogQHJldHVybnMge0NvbXBsZXh9IFRoZSByZXN1bHQgb2YgaW52ZXJzZSBjb3RhbmdlbnQgZnVuY3Rpb25cclxuICovXG5mdW5jdGlvbiBhY290KG51bSkge1xuICByZXR1cm4gdGhpcy5hdGFuKHRoaXMuaW52ZXJzZShudW0pKTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBhY290OyIsIlwidXNlIHN0cmljdFwiO1xuXG4vKipcclxuICogQ2FsY3VsYXRlcyB0aGUgaW52ZXJzZSBjb3NlY2FudCBvZiBhIENvbXBsZXggTnVtYmVyLlxyXG4gKiBUaGUgZG9tYWluIG9mIHRoaXMgZnVuY3Rpb24gaXMgQyAvIHsgMCB9Ljxicj48YnI+XHJcbiAqIFxyXG4gKiBJZiB0aGUgYXJndW1lbnQgaXMgb3V0IG9mIGl0cyBkb21haW4sIGl0IHJldHVybnMgQ29tcGxleC5OYU4uXHJcbiAqIEBtZW1iZXJvZiBDb21wbGV4XHJcbiAqIEBzdGF0aWNcclxuICogQHBhcmFtIHtDb21wbGV4fSBudW0gLSBBbnkgQ29tcGxleCBOdW1iZXIgZXhjZXB0IDBcclxuICogQHJldHVybnMge0NvbXBsZXh9IFRoZSByZXN1bHQgb2YgaW52ZXJzZSBjb3NlY2FudCBmdW5jdGlvblxyXG4gKi9cbmZ1bmN0aW9uIGFjc2MobnVtKSB7XG4gIHJldHVybiB0aGlzLmFzaW4odGhpcy5pbnZlcnNlKG51bSkpO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IGFjc2M7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbi8qKlxyXG4gKiBDYWxjdWxhdGVzIHRoZSBzdW0gb2YgdHdvIENvbXBsZXggTnVtYmVyLlxyXG4gKiBAbWVtYmVyb2YgQ29tcGxleFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7Q29tcGxleH0gbnVtMSAtIFRoZSBDb21wbGV4IE51bWJlciBvbiB0aGUgbGVmdCBvZiAnKycgb3BlcmF0b3IuXHJcbiAqIEBwYXJhbSB7Q29tcGxleH0gbnVtMiAtIFRoZSBDb21wbGV4IE51bWJlciBvbiB0aGUgcmlnaHQgb2YgJysnIG9wZXJhdG9yLlxyXG4gKiBAcmV0dXJucyB7Q29tcGxleH0gVGhlIHN1bSBvZiB0d28gQ29tcGxleCBOdW1iZXJzXHJcbiAqL1xuZnVuY3Rpb24gYWRkKG51bTEsIG51bTIpIHtcbiAgaWYgKCEobnVtMSBpbnN0YW5jZW9mIHRoaXMpIHx8ICEobnVtMiBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgcmV0dXJuIHRoaXMuTmFOO1xuICB9XG5cbiAgcmV0dXJuIG5ldyB0aGlzKG51bTEucmUgKyBudW0yLnJlLCBudW0xLmltICsgbnVtMi5pbSk7XG59XG5cbm1vZHVsZS5leHBvcnRzID0gYWRkOyIsIlwidXNlIHN0cmljdFwiO1xuXG4vKipcclxuICogQ2FsY3VsYXRlcyB0aGUgaW52ZXJzZSBzZWNhbnQgb2YgYSBDb21wbGV4IE51bWJlci5cclxuICogVGhlIGRvbWFpbiBvZiB0aGlzIGZ1bmN0aW9uIGlzIEMgLyB7IDAgfS48YnI+PGJyPlxyXG4gKiBcclxuICogSWYgdGhlIGFyZ3VtZW50IGlzIG91dCBvZiBpdHMgZG9tYWluLCBpdCByZXR1cm5zIENvbXBsZXguTmFOLlxyXG4gKiBAbWVtYmVyb2YgQ29tcGxleFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7Q29tcGxleH0gbnVtIC0gQW55IENvbXBsZXggTnVtYmVyIGV4Y2VwdCAwXHJcbiAqIEByZXR1cm5zIHtDb21wbGV4fSBUaGUgcmVzdWx0IG9mIGludmVyc2Ugc2VjYW50IGZ1bmN0aW9uXHJcbiAqL1xuZnVuY3Rpb24gYXNlYyhudW0pIHtcbiAgcmV0dXJuIHRoaXMuYWNvcyh0aGlzLmludmVyc2UobnVtKSk7XG59XG5cbm1vZHVsZS5leHBvcnRzID0gYXNlYzsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIENhbGN1bGF0ZXMgdGhlIGludmVyc2Ugc2luZSBvZiBhIENvbXBsZXggTnVtYmVyLlxyXG4gKiBAbWVtYmVyb2YgQ29tcGxleFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7Q29tcGxleH0gbnVtIC0gQW55IENvbXBsZXggTnVtYmVyXHJcbiAqIEByZXR1cm5zIHtDb21wbGV4fSBUaGUgcmVzdWx0IG9mIGludmVyc2Ugc2luZSBmdW5jdGlvblxyXG4gKi9cbmZ1bmN0aW9uIGFzaW4obnVtKSB7XG4gIHJldHVybiB0aGlzLm11bHRpcGx5KG5ldyB0aGlzKDAsIC0xKSwgdGhpcy5sb2codGhpcy5hZGQodGhpcy5tdWx0aXBseShuZXcgdGhpcygwLCAxKSwgbnVtKSwgdGhpcy5wb3codGhpcy5zdWJ0cmFjdCh0aGlzLk9ORSwgdGhpcy5wb3cobnVtLCAyKSksIDAuNSkpKSk7XG59XG5cbm1vZHVsZS5leHBvcnRzID0gYXNpbjsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIENhbGN1bGF0ZXMgdGhlIGludmVyc2UgdGFuZ2VudCBvZiBhIENvbXBsZXggTnVtYmVyLlxyXG4gKiBUaGUgZG9tYWluIG9mIHRoaXMgZnVuY3Rpb24gaXMgQyAvIHsgaSAsIC1pIH0uPGJyPjxicj5cclxuICogXHJcbiAqIElmIHRoZSBhcmd1bWVudCBpcyBvdXQgb2YgaXRzIGRvbWFpbiwgaXQgcmV0dXJucyBDb21wbGV4Lk5hTi5cclxuICogQG1lbWJlcm9mIENvbXBsZXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge0NvbXBsZXh9IG51bSAtIEFueSBDb21wbGV4IE51bWJlciBleGNlcHQgaSBhbmQgLWlcclxuICogQHJldHVybnMge0NvbXBsZXh9IFRoZSByZXN1bHQgb2YgaW52ZXJzZSB0YW5nZW50IGZ1bmN0aW9uXHJcbiAqL1xuZnVuY3Rpb24gYXRhbihudW0pIHtcbiAgcmV0dXJuIHRoaXMubXVsdGlwbHkobmV3IHRoaXMoMCwgMSAvIDIpLCB0aGlzLnN1YnRyYWN0KHRoaXMubG9nKHRoaXMuc3VidHJhY3QodGhpcy5PTkUsIHRoaXMubXVsdGlwbHkobmV3IHRoaXMoMCwgMSksIG51bSkpKSwgdGhpcy5sb2codGhpcy5hZGQodGhpcy5PTkUsIHRoaXMubXVsdGlwbHkobmV3IHRoaXMoMCwgMSksIG51bSkpKSkpO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IGF0YW47IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbi8qKlxyXG4gKiBDYWxjdWxhdGVzIHRoZSBjb21wbGV4IGNvbmp1Z2F0ZSBvZiB0aGUgQ29tcGxleCBOdW1iZXIuXHJcbiAqIEBtZW1iZXJvZiBDb21wbGV4XHJcbiAqIEBzdGF0aWNcclxuICogQHBhcmFtIHtDb21wbGV4fSBudW0gLSBDb21wbGV4IG51bWJlclxyXG4gKiBAcmV0dXJucyB7Q29tcGxleH0gVGhlIGNvbXBsZXggY29uanVnYXRlIG9mIHRoZSBDb21wbGV4IE51bWJlclxyXG4gKi9cbmZ1bmN0aW9uIGNvbmp1Z2F0ZShudW0pIHtcbiAgaWYgKCEobnVtIGluc3RhbmNlb2YgdGhpcykpIHtcbiAgICByZXR1cm4gdGhpcy5OYU47XG4gIH1cblxuICByZXR1cm4gbmV3IHRoaXMobnVtLmdldFJlYWwoKSwgbnVtLmdldEltYWdpbmFyeSgpICogLTEpO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IGNvbmp1Z2F0ZTsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIENhbGN1bGF0ZXMgdGhlIGNvc2luZSBvZiBhIENvbXBsZXggTnVtYmVyLlxyXG4gKiBAbWVtYmVyb2YgQ29tcGxleFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7Q29tcGxleH0gbnVtIC0gQW55IENvbXBsZXggTnVtYmVyXHJcbiAqIEByZXR1cm5zIHtDb21wbGV4fSBUaGUgcmVzdWx0IG9mIGNvc2luZSBmdW5jdGlvblxyXG4gKi9cbmZ1bmN0aW9uIGNvcyhudW0pIHtcbiAgaWYgKCEobnVtIGluc3RhbmNlb2YgdGhpcykpIHtcbiAgICByZXR1cm4gdGhpcy5OYU47XG4gIH1cblxuICB2YXIgYSA9IG51bS5nZXRSZWFsKCk7XG4gIHZhciBiID0gbnVtLmdldEltYWdpbmFyeSgpO1xuICByZXR1cm4gbmV3IHRoaXMoTWF0aC5jb3MoYSkgKiBNYXRoLmNvc2goYiksIE1hdGguc2luKGEpICogTWF0aC5zaW5oKGIpICogLTEpO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IGNvczsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIENhbGN1bGF0ZXMgdGhlIGNvdGFuZ2VudCBvZiBhIENvbXBsZXggTnVtYmVyLlxyXG4gKiBUaGUgZG9tYWluIG9mIHRoaXMgZnVuY3Rpb24gaXMgQyAvIHsga8+ALzIgOiBrIGlzIGFueSBpbnRlZ2VyIH0uPGJyPjxicj5cclxuICogXHJcbiAqIElmIHRoZSBhcmd1bWVudCBpcyBvdXQgb2YgaXRzIGRvbWFpbiwgaXQgcmV0dXJucyBDb21wbGV4Lk5hTi5cclxuICogQG1lbWJlcm9mIENvbXBsZXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge0NvbXBsZXh9IG51bSAtIEFueSBDb21wbGV4IE51bWJlciB3aGljaCBpcyBub3QgdGhlIG11bHRpcGxlIG9mIM+ALzJcclxuICogQHJldHVybnMge0NvbXBsZXh9IFRoZSByZXN1bHQgb2YgY290YW5nZW50IGZ1bmN0aW9uXHJcbiAqL1xuZnVuY3Rpb24gY290KG51bSkge1xuICByZXR1cm4gdGhpcy5kaXZpZGUodGhpcy5PTkUsIHRoaXMudGFuKG51bSkpO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IGNvdDsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIENhbGN1bGF0ZXMgdGhlIGNvc2VjYW50IG9mIGEgQ29tcGxleCBOdW1iZXIuXHJcbiAqIFRoZSBkb21haW4gb2YgdGhpcyBmdW5jdGlvbiBpcyBDIC8geyBrz4AgOiBrIGlzIGFueSBpbnRlZ2VyIH0uPGJyPjxicj5cclxuICogXHJcbiAqIElmIHRoZSBhcmd1bWVudCBpcyBvdXQgb2YgaXRzIGRvbWFpbiwgaXQgcmV0dXJucyBDb21wbGV4Lk5hTi5cclxuICogQG1lbWJlcm9mIENvbXBsZXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge0NvbXBsZXh9IG51bSAtIEFueSBDb21wbGV4IE51bWJlciB3aGljaCBpcyBub3QgdGhlIG11bHRpcGxlIG9mIM+AXHJcbiAqIEByZXR1cm5zIHtDb21wbGV4fSBUaGUgcmVzdWx0IG9mIGNvc2VjYW50IGZ1bmN0aW9uXHJcbiAqL1xuZnVuY3Rpb24gY3NjKG51bSkge1xuICByZXR1cm4gdGhpcy5kaXZpZGUodGhpcy5PTkUsIHRoaXMuc2luKG51bSkpO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IGNzYzsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIENhbGN1bGF0ZXMgdGhlIHF1b3RpZW50IG9mIHR3byBDb21wbGV4IE51bWJlci48YnI+PGJyPlxyXG4gKiBcclxuICogTm90ZSB0aGF0IGlmIHRoZSBkZW5vbWluYXRvciBpcyBjb25zaWRlcmVkIGFzIDAsXHJcbiAqIHJldHVybnMgQ29tcGxleC5OYU4gaW5zdGVhZCBvZiBJbmZpbml0eS5cclxuICogQG1lbWJlcm9mIENvbXBsZXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge0NvbXBsZXh9IG51bTEgLSBUaGUgQ29tcGxleCBOdW1iZXIgb24gdGhlIGxlZnQgb2YgJy8nIG9wZXJhdG9yLlxyXG4gKiBAcGFyYW0ge0NvbXBsZXh9IG51bTIgLSBUaGUgQ29tcGxleCBOdW1iZXIgb24gdGhlIHJpZ2h0IG9mICcvJyBvcGVyYXRvci5cclxuICogQHJldHVybnMge0NvbXBsZXh9IFRoZSBxdW90aWVudCBvZiB0d28gQ29tcGxleCBOdW1iZXJzXHJcbiAqL1xuZnVuY3Rpb24gZGl2aWRlKG51bTEsIG51bTIpIHtcbiAgaWYgKCEobnVtMSBpbnN0YW5jZW9mIHRoaXMpIHx8ICEobnVtMiBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgcmV0dXJuIHRoaXMuTmFOO1xuICB9XG5cbiAgdmFyIGEgPSBudW0xLnJlO1xuICB2YXIgYiA9IG51bTEuaW07XG4gIHZhciBjID0gbnVtMi5yZTtcbiAgdmFyIGQgPSBudW0yLmltO1xuXG4gIGlmIChNYXRoLmFicyhjKSA8IHRoaXMuRVBTSUxPTiAmJiBNYXRoLmFicyhkKSA8IHRoaXMuRVBTSUxPTikge1xuICAgIHJldHVybiB0aGlzLk5hTjtcbiAgfVxuXG4gIHZhciBkZW5vbWluYXRvciA9IE1hdGgucG93KGMsIDIpICsgTWF0aC5wb3coZCwgMik7XG4gIHJldHVybiBuZXcgdGhpcygoYSAqIGMgKyBiICogZCkgLyBkZW5vbWluYXRvciwgKGIgKiBjIC0gYSAqIGQpIC8gZGVub21pbmF0b3IpO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IGRpdmlkZTsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIENhbGN1bGF0ZXMgdGhlIGV4cG9uZW50aWFsIGZ1bmN0aW9uIHdpdGggYmFzZSBFLlxyXG4gKiBAbWVtYmVyb2YgQ29tcGxleFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7Q29tcGxleH0gbnVtIC0gRXhwb25lbnRcclxuICogQHJldHVybnMge0NvbXBsZXh9IFRoZSB2YWx1ZSBvZiBFIHRvIHRoZSBwb3dlciBvZiBudW1cclxuICovXG5mdW5jdGlvbiBleHAobnVtKSB7XG4gIGlmICghKG51bSBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgcmV0dXJuIHRoaXMuTmFOO1xuICB9XG5cbiAgdmFyIHJlID0gbnVtLmdldFJlYWwoKTtcbiAgdmFyIHRoZXRhID0gbnVtLmdldEltYWdpbmFyeSgpO1xuICB2YXIgciA9IE1hdGguZXhwKHJlKTtcbiAgcmV0dXJuIG5ldyB0aGlzKHIgKiBNYXRoLmNvcyh0aGV0YSksIHIgKiBNYXRoLnNpbih0aGV0YSkpO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IGV4cDsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIENhbGN1bGF0ZXMgdGhlIGludmVyc2Ugb2YgdGhlIENvbXBsZXggTnVtYmVyLlxyXG4gKiBAbWVtYmVyb2YgQ29tcGxleFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7Q29tcGxleH0gbnVtIC0gQ29tcGxleCBOdW1iZXJcclxuICogQHJldHVybnMge251bWJlcn0gSW52ZXJzZSBvZiB0aGUgQ29tcGxleCBOdW1iZXJcclxuICovXG5mdW5jdGlvbiBpbnZlcnNlKG51bSkge1xuICBpZiAoIShudW0gaW5zdGFuY2VvZiB0aGlzKSkge1xuICAgIHJldHVybiB0aGlzLk5hTjtcbiAgfVxuXG4gIHJldHVybiB0aGlzLmRpdmlkZSh0aGlzLk9ORSwgbnVtKTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBpbnZlcnNlOyIsIlwidXNlIHN0cmljdFwiO1xuXG4vKipcclxuICogRGV0ZXJtaW5lcyB3aGV0aGVyIHR3byBDb21wbGV4IE51bWJlcnMgYXJlIGNvbnNpZGVyZWQgYXMgaWRlbnRpY2FsLjxicj48YnI+XHJcbiAqIFxyXG4gKiBUd28gQ29tcGxleCBOdW1iZXJzIGFyZSBjb25zaWRlcmVkIGFzIGlkZW50aWNhbCBpZiBlaXRoZXJcclxuICogYm90aCBhcmUgTmFOIG9yIGJvdGggcmVhbCBhbmQgaW1hZ2luYXJ5IHBhcnRzIGFyZSBleHRyZW1lbHkgY2xvc2VkLjxicj48YnI+XHJcbiAqIFxyXG4gKiBUaGUgdGVzdCBjcml0ZXJpb24gaXMgTWF0aC5hYnMoeCAtIHkpIDwgMSAvICgxMCAqKiBkaWdpdCAqIDIpLlxyXG4gKiBGb3IgZGVmYXVsdCB2YWx1ZSAxNSwgaXQgc2hvdWxkIGJlIDVlLTE2LlxyXG4gKiBUaGF0IG1lYW5zIGlmIHRoZSBkaWZmZXJlbmNlIG9mIHR3byBudW1iZXJzIGlzIGxlc3MgdGhhbiA1ZS0xNixcclxuICogdGhleSBhcmUgY29uc2lkZXJlZCBhcyBzYW1lIHZhbHVlLlxyXG4gKiBAbWVtYmVyb2YgQ29tcGxleFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7Q29tcGxleH0gbnVtMSAtIENvbXBsZXggTnVtYmVyXHJcbiAqIEBwYXJhbSB7Q29tcGxleH0gbnVtMiAtIENvbXBsZXggTnVtYmVyXHJcbiAqIEBwYXJhbSB7bnVtYmVyfSBbZGlnaXQ9MTVdIC0gTnVtYmVyIG9mIHNpZ25pZmljYW50IGRpZ2l0c1xyXG4gKiBAcmV0dXJucyB7Ym9vbGVhbn0gUmV0dXJucyB0cnVlIGlmIHR3byBDb21wbGV4IE51bWJlcnMgYXJlIGNvbnNpZGVyZWQgYXMgaWRlbnRpY2FsXHJcbiAqL1xuZnVuY3Rpb24gaXNFcXVhbChudW0xLCBudW0yKSB7XG4gIHZhciBkaWdpdCA9IGFyZ3VtZW50cy5sZW5ndGggPiAyICYmIGFyZ3VtZW50c1syXSAhPT0gdW5kZWZpbmVkID8gYXJndW1lbnRzWzJdIDogMTU7XG5cbiAgaWYgKCEobnVtMSBpbnN0YW5jZW9mIHRoaXMpIHx8ICEobnVtMiBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG5cbiAgaWYgKCFOdW1iZXIuaXNJbnRlZ2VyKGRpZ2l0KSB8fCBkaWdpdCA8IDApIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoJ0ludmFsaWQgYXJndW1lbnQ6IEV4cGVjdGVkIGEgbm9uLW5lZ2F0aXZlIGludGVnZXIgZGlnaXQnKTtcbiAgfVxuXG4gIHZhciBFUFNJTE9OID0gMSAvIChNYXRoLnBvdygxMCwgZGlnaXQpICogMik7XG4gIHZhciBhID0gbnVtMS5nZXRSZWFsKCk7XG4gIHZhciBiID0gbnVtMS5nZXRJbWFnaW5hcnkoKTtcbiAgdmFyIGMgPSBudW0yLmdldFJlYWwoKTtcbiAgdmFyIGQgPSBudW0yLmdldEltYWdpbmFyeSgpO1xuXG4gIGlmIChOdW1iZXIuaXNOYU4oYSkgJiYgTnVtYmVyLmlzTmFOKGIpICYmIE51bWJlci5pc05hTihjKSAmJiBOdW1iZXIuaXNOYU4oZCkpIHtcbiAgICByZXR1cm4gdHJ1ZTtcbiAgfVxuXG4gIHJldHVybiBNYXRoLmFicyhhIC0gYykgPCBFUFNJTE9OICYmIE1hdGguYWJzKGIgLSBkKSA8IEVQU0lMT047XG59XG5cbm1vZHVsZS5leHBvcnRzID0gaXNFcXVhbDsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIERldGVybWluZXMgd2hldGhlciB0aGUgQ29tcGxleCBOdW1iZXIgaXMgTmFOIG9yIG5vdC5cclxuICogQG1lbWJlcm9mIENvbXBsZXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge0NvbXBsZXh9IG51bSAtIEFueSBDb21wbGV4IG51bWJlclxyXG4gKiBAcmV0dXJucyB7Ym9vbGVhbn0gUmV0dXJucyB0cnVlIGlmIG9uZSBvZiByZWFsIGFuZCBpbWFnaW5hcnkgcGFydCBhcmUgTmFOXHJcbiAqL1xuZnVuY3Rpb24gaXNOYU4obnVtKSB7XG4gIGlmICghKG51bSBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG5cbiAgdmFyIHJlID0gbnVtLmdldFJlYWwoKTtcbiAgdmFyIGltID0gbnVtLmdldEltYWdpbmFyeSgpO1xuXG4gIGlmIChOdW1iZXIuaXNOYU4ocmUpIHx8IE51bWJlci5pc05hTihpbSkpIHtcbiAgICByZXR1cm4gdHJ1ZTtcbiAgfVxuXG4gIHJldHVybiBmYWxzZTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBpc05hTjsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIENhbGN1bGF0ZXMgdGhlIG5hdHVyYWwgbG9nIG9mIHRoZSBDb21wbGV4IE51bWJlci48YnI+PGJyPlxyXG4gKiBcclxuICogTm90ZSB0aGF0IGNvbXBsZXggbG9nIGlzIGEgbXVsdGl2YWx1ZWQgZnVuY3Rpb24sXHJcbiAqIGFuZCB0aGlzIGZ1bmN0aW9uIG9ubHkgcHJvdmlkZXMgdGhlIHByaW5jaXBhbCB2YWx1ZSBieVxyXG4gKiByZXN0cmljdGluZyB0aGUgaW1hZ2luYXJ5IHBhcnQgdG8gdGhlIGludGVydmFsIFswLCAyz4ApLlxyXG4gKiBAbWVtYmVyb2YgQ29tcGxleFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7Q29tcGxleH0gbnVtIC0gQ29tcGxleCBOdW1iZXJcclxuICogQHJldHVybnMge251bWJlcn0gTmF0dXJhbCBsb2cgb2YgdGhlIENvbXBsZXggTnVtYmVyXHJcbiAqL1xuZnVuY3Rpb24gbG9nKG51bSkge1xuICBpZiAoIShudW0gaW5zdGFuY2VvZiB0aGlzKSkge1xuICAgIHJldHVybiB0aGlzLk5hTjtcbiAgfVxuXG4gIHZhciByID0gbnVtLmdldE1vZHVsdXMoKTtcbiAgdmFyIHRoZXRhID0gbnVtLmdldEFyZ3VtZW50KCk7XG5cbiAgaWYgKHIgPCB0aGlzLkVQU0lMT04gfHwgdGhldGEgPT09IHVuZGVmaW5lZCkge1xuICAgIHJldHVybiB0aGlzLk5hTjtcbiAgfVxuXG4gIHJldHVybiBuZXcgdGhpcyhNYXRoLmxvZyhyKSwgdGhldGEpO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IGxvZzsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIENhbGN1bGF0ZXMgdGhlIHByb2R1Y3Qgb2YgdHdvIENvbXBsZXggTnVtYmVyLlxyXG4gKiBAbWVtYmVyb2YgQ29tcGxleFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7Q29tcGxleH0gbnVtMSAtIFRoZSBDb21wbGV4IE51bWJlciBvbiB0aGUgbGVmdCBvZiAnKicgb3BlcmF0b3IuXHJcbiAqIEBwYXJhbSB7Q29tcGxleH0gbnVtMiAtIFRoZSBDb21wbGV4IE51bWJlciBvbiB0aGUgcmlnaHQgb2YgJyonIG9wZXJhdG9yLlxyXG4gKiBAcmV0dXJucyB7Q29tcGxleH0gVGhlIHByb2R1Y3Qgb2YgdHdvIENvbXBsZXggTnVtYmVyc1xyXG4gKi9cbmZ1bmN0aW9uIG11bHRpcGx5KG51bTEsIG51bTIpIHtcbiAgaWYgKCEobnVtMSBpbnN0YW5jZW9mIHRoaXMpIHx8ICEobnVtMiBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgcmV0dXJuIHRoaXMuTmFOO1xuICB9XG5cbiAgdmFyIGEgPSBudW0xLnJlO1xuICB2YXIgYiA9IG51bTEuaW07XG4gIHZhciBjID0gbnVtMi5yZTtcbiAgdmFyIGQgPSBudW0yLmltO1xuICByZXR1cm4gbmV3IHRoaXMoYSAqIGMgLSBiICogZCwgYSAqIGQgKyBiICogYyk7XG59XG5cbm1vZHVsZS5leHBvcnRzID0gbXVsdGlwbHk7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbi8qKlxyXG4gKiBDYWxjdWxhdGVzIHRoZSBwb3dlciBvZiB0aGUgQ29tcGxleCBOdW1iZXIuXHJcbiAqIFRoZSBleHBvbmVudCBjYW4gYmUgYW55IHJlYWwgbnVtYmVyIG9yIENvbXBsZXggTnVtYmVyPGJyPjxicj5cclxuICogXHJcbiAqIFlvdSBjYW4gZmluZCB0aGUgay10aCByb290IG9mIGNvbXBsZXggbnVtYmVyIGJ5IHNldHRpbmcgdGhlIGV4cG9uZW50IHRvIDEgLyBrLlxyXG4gKiBCdXQgeW91IHNob3VsZCBrbm93IHRoYXQgaXQgb25seSByZXR1cm5zIG9uZSBvdXQgb2YgayBwb3NzaWJsZSBzb2x1dGlvbnMuXHJcbiAqIEBtZW1iZXJvZiBDb21wbGV4XHJcbiAqIEBzdGF0aWNcclxuICogQHBhcmFtIHtDb21wbGV4fSBudW0gLSBCYXNlXHJcbiAqIEBwYXJhbSB7Q29tcGxleHxudW1iZXJ9IG4gLSBFeHBvbmVudFxyXG4gKiBAcmV0dXJucyB7Q29tcGxleH0gVGhlIHJlc3VsdCBvZiB0aGUgZXhwb25lbnRpYXRpb25cclxuICovXG5mdW5jdGlvbiBwb3cobnVtLCBuKSB7XG4gIGlmICghKG51bSBpbnN0YW5jZW9mIHRoaXMpIHx8IHR5cGVvZiBuICE9PSAnbnVtYmVyJyAmJiAhKG4gaW5zdGFuY2VvZiB0aGlzKSkge1xuICAgIHJldHVybiB0aGlzLk5hTjtcbiAgfVxuXG4gIGlmICh0eXBlb2YgbiA9PT0gJ251bWJlcicpIHtcbiAgICBpZiAoIU51bWJlci5pc0Zpbml0ZShuKSB8fCBOdW1iZXIuaXNOYU4obikpIHtcbiAgICAgIHJldHVybiB0aGlzLk5hTjtcbiAgICB9XG5cbiAgICBpZiAobiA9PT0gMCkge1xuICAgICAgcmV0dXJuIHRoaXMuT05FO1xuICAgIH1cblxuICAgIGlmICh0aGlzLmlzRXF1YWwobnVtLCB0aGlzLlpFUk8pKSB7XG4gICAgICByZXR1cm4gdGhpcy5aRVJPO1xuICAgIH1cblxuICAgIHJldHVybiB0aGlzLmV4cCh0aGlzLm11bHRpcGx5KG5ldyB0aGlzKG4sIDApLCB0aGlzLmxvZyhudW0pKSk7XG4gIH1cblxuICBpZiAobiBpbnN0YW5jZW9mIHRoaXMpIHtcbiAgICByZXR1cm4gdGhpcy5leHAodGhpcy5tdWx0aXBseShuLCB0aGlzLmxvZyhudW0pKSk7XG4gIH1cblxuICByZXR1cm4gdGhpcy5OYU47XG59XG5cbm1vZHVsZS5leHBvcnRzID0gcG93OyIsIlwidXNlIHN0cmljdFwiO1xuXG4vKipcclxuICogQ2FsY3VsYXRlcyB0aGUgc2VjYW50IG9mIGEgQ29tcGxleCBOdW1iZXIuXHJcbiAqIFRoZSBkb21haW4gb2YgdGhpcyBmdW5jdGlvbiBpcyBDIC8geyAoayArIDAuNSnPgCA6IGsgaXMgYW55IGludGVnZXIgfS48YnI+PGJyPlxyXG4gKiBcclxuICogSWYgdGhlIGFyZ3VtZW50IGlzIG91dCBvZiBpdHMgZG9tYWluLCBpdCByZXR1cm5zIENvbXBsZXguTmFOLlxyXG4gKiBAbWVtYmVyb2YgQ29tcGxleFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7Q29tcGxleH0gbnVtIC0gQW55IENvbXBsZXggTnVtYmVyIHdoaWNoIGlzIG5vdCBpbiB0aGUgZm9ybSBvZiAoayArIDAuNSnPgFxyXG4gKiBAcmV0dXJucyB7Q29tcGxleH0gVGhlIHJlc3VsdCBvZiBzZWNhbnQgZnVuY3Rpb25cclxuICovXG5mdW5jdGlvbiBzZWMobnVtKSB7XG4gIHJldHVybiB0aGlzLmRpdmlkZSh0aGlzLk9ORSwgdGhpcy5jb3MobnVtKSk7XG59XG5cbm1vZHVsZS5leHBvcnRzID0gc2VjOyIsIlwidXNlIHN0cmljdFwiO1xuXG4vKipcclxuICogQ2FsY3VsYXRlcyB0aGUgc2luZSBvZiBhIENvbXBsZXggTnVtYmVyLlxyXG4gKiBAbWVtYmVyb2YgQ29tcGxleFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7Q29tcGxleH0gbnVtIC0gQW55IENvbXBsZXggTnVtYmVyXHJcbiAqIEByZXR1cm5zIHtDb21wbGV4fSBUaGUgcmVzdWx0IG9mIHNpbmUgZnVuY3Rpb25cclxuICovXG5mdW5jdGlvbiBzaW4obnVtKSB7XG4gIGlmICghKG51bSBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgcmV0dXJuIHRoaXMuTmFOO1xuICB9XG5cbiAgdmFyIGEgPSBudW0uZ2V0UmVhbCgpO1xuICB2YXIgYiA9IG51bS5nZXRJbWFnaW5hcnkoKTtcbiAgcmV0dXJuIG5ldyB0aGlzKE1hdGguc2luKGEpICogTWF0aC5jb3NoKGIpLCBNYXRoLmNvcyhhKSAqIE1hdGguc2luaChiKSk7XG59XG5cbm1vZHVsZS5leHBvcnRzID0gc2luOyIsIlwidXNlIHN0cmljdFwiO1xuXG4vKipcclxuICogQ2FsY3VsYXRlcyB0aGUgZGlmZmVyZW5jZSBvZiB0d28gQ29tcGxleCBOdW1iZXIuXHJcbiAqIEBtZW1iZXJvZiBDb21wbGV4XHJcbiAqIEBzdGF0aWNcclxuICogQHBhcmFtIHtDb21wbGV4fSBudW0xIC0gVGhlIENvbXBsZXggTnVtYmVyIG9uIHRoZSBsZWZ0IG9mICctJyBvcGVyYXRvci5cclxuICogQHBhcmFtIHtDb21wbGV4fSBudW0yIC0gVGhlIENvbXBsZXggTnVtYmVyIG9uIHRoZSByaWdodCBvZiAnLScgb3BlcmF0b3IuXHJcbiAqIEByZXR1cm5zIHtDb21wbGV4fSBUaGUgZGlmZmVyZW5jZSBvZiB0d28gQ29tcGxleCBOdW1iZXJzXHJcbiAqL1xuZnVuY3Rpb24gc3VidHJhY3QobnVtMSwgbnVtMikge1xuICBpZiAoIShudW0xIGluc3RhbmNlb2YgdGhpcykgfHwgIShudW0yIGluc3RhbmNlb2YgdGhpcykpIHtcbiAgICByZXR1cm4gdGhpcy5OYU47XG4gIH1cblxuICByZXR1cm4gbmV3IHRoaXMobnVtMS5yZSAtIG51bTIucmUsIG51bTEuaW0gLSBudW0yLmltKTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBzdWJ0cmFjdDsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIENhbGN1bGF0ZXMgdGhlIHRhbmdlbnQgb2YgYSBDb21wbGV4IE51bWJlci5cclxuICogVGhlIGRvbWFpbiBvZiB0aGlzIGZ1bmN0aW9uIGlzIEMgLyB7IChrICsgMC41Kc+AIDogayBpcyBhbnkgaW50ZWdlciB9Ljxicj48YnI+XHJcbiAqIFxyXG4gKiBJZiB0aGUgYXJndW1lbnQgaXMgb3V0IG9mIGl0cyBkb21haW4sIGl0IHJldHVybnMgQ29tcGxleC5OYU4uXHJcbiAqIEBtZW1iZXJvZiBDb21wbGV4XHJcbiAqIEBzdGF0aWNcclxuICogQHBhcmFtIHtDb21wbGV4fSBudW0gLSBBbnkgQ29tcGxleCBOdW1iZXIgd2hpY2ggaXMgbm90IGluIHRoZSBmb3JtIG9mIChrICsgMC41Kc+AXHJcbiAqIEByZXR1cm5zIHtDb21wbGV4fSBUaGUgcmVzdWx0IG9mIHRhbmdlbnQgZnVuY3Rpb25cclxuICovXG5mdW5jdGlvbiB0YW4obnVtKSB7XG4gIHJldHVybiB0aGlzLmRpdmlkZSh0aGlzLnNpbihudW0pLCB0aGlzLmNvcyhudW0pKTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSB0YW47IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbmZ1bmN0aW9uIF90eXBlb2Yob2JqKSB7IFwiQGJhYmVsL2hlbHBlcnMgLSB0eXBlb2ZcIjsgaWYgKHR5cGVvZiBTeW1ib2wgPT09IFwiZnVuY3Rpb25cIiAmJiB0eXBlb2YgU3ltYm9sLml0ZXJhdG9yID09PSBcInN5bWJvbFwiKSB7IF90eXBlb2YgPSBmdW5jdGlvbiBfdHlwZW9mKG9iaikgeyByZXR1cm4gdHlwZW9mIG9iajsgfTsgfSBlbHNlIHsgX3R5cGVvZiA9IGZ1bmN0aW9uIF90eXBlb2Yob2JqKSB7IHJldHVybiBvYmogJiYgdHlwZW9mIFN5bWJvbCA9PT0gXCJmdW5jdGlvblwiICYmIG9iai5jb25zdHJ1Y3RvciA9PT0gU3ltYm9sICYmIG9iaiAhPT0gU3ltYm9sLnByb3RvdHlwZSA/IFwic3ltYm9sXCIgOiB0eXBlb2Ygb2JqOyB9OyB9IHJldHVybiBfdHlwZW9mKG9iaik7IH1cblxuLyoqXHJcbiAqIENyZWF0ZXMgYSBuZXcgQ29tcGxleCBOdW1iZXIuXHJcbiAqIEBuYW1lc3BhY2UgQ29tcGxleFxyXG4gKiBAY2xhc3NcclxuICogQHBhcmFtIHtudW1iZXJ9IGFyZzEgLSBUaGUgcmVhbCBwYXJ0IG9mIHRoZSBDb21wbGV4IE51bWJlclxyXG4gKiBAcGFyYW0ge251bWJlcn0gYXJnMiAtIFRoZSBpbWFnaW5hcnkgcGFydCBvZiB0aGUgQ29tcGxleCBOdW1iZXJcclxuICovXG5mdW5jdGlvbiBDb21wbGV4KGFyZzEsIGFyZzIpIHtcbiAgdmFyIHR5cGUxID0gX3R5cGVvZihhcmcxKTtcblxuICB2YXIgdHlwZTIgPSBfdHlwZW9mKGFyZzIpO1xuXG4gIGlmICh0eXBlMSA9PT0gJ251bWJlcicgJiYgdHlwZTIgPT09ICd1bmRlZmluZWQnKSB7XG4gICAgaWYgKE51bWJlci5pc05hTihhcmcxKSB8fCAhTnVtYmVyLmlzRmluaXRlKGFyZzEpKSB7XG4gICAgICB0aGlzLnJlID0gTmFOO1xuICAgICAgdGhpcy5pbSA9IE5hTjtcbiAgICAgIHJldHVybiB0aGlzO1xuICAgIH1cblxuICAgIHRoaXMucmUgPSBhcmcxO1xuICAgIHRoaXMuaW0gPSAwO1xuICAgIHJldHVybiB0aGlzO1xuICB9XG5cbiAgaWYgKHR5cGUxID09PSAnbnVtYmVyJyAmJiB0eXBlMiA9PT0gJ251bWJlcicpIHtcbiAgICBpZiAoTnVtYmVyLmlzTmFOKGFyZzEpIHx8IE51bWJlci5pc05hTihhcmcyKSB8fCAhTnVtYmVyLmlzRmluaXRlKGFyZzEpIHx8ICFOdW1iZXIuaXNGaW5pdGUoYXJnMikpIHtcbiAgICAgIHRoaXMucmUgPSBOYU47XG4gICAgICB0aGlzLmltID0gTmFOO1xuICAgICAgcmV0dXJuIHRoaXM7XG4gICAgfVxuXG4gICAgdGhpcy5yZSA9IGFyZzE7XG4gICAgdGhpcy5pbSA9IGFyZzI7XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICB0aGlzLnJlID0gTmFOO1xuICB0aGlzLmltID0gTmFOO1xuICByZXR1cm4gdGhpcztcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBDb21wbGV4O1xuQ29tcGxleC5wcm90b3R5cGUuZ2V0UmVhbCA9IHJlcXVpcmUoJy4vY29yZS9pbnN0YW5jZS9nZXRSZWFsJyk7XG5Db21wbGV4LnByb3RvdHlwZS5nZXRJbWFnaW5hcnkgPSByZXF1aXJlKCcuL2NvcmUvaW5zdGFuY2UvZ2V0SW1hZ2luYXJ5Jyk7XG5Db21wbGV4LnByb3RvdHlwZS5nZXRNb2R1bHVzID0gcmVxdWlyZSgnLi9jb3JlL2luc3RhbmNlL2dldE1vZHVsdXMnKTtcbkNvbXBsZXgucHJvdG90eXBlLmdldEFyZ3VtZW50ID0gcmVxdWlyZSgnLi9jb3JlL2luc3RhbmNlL2dldEFyZ3VtZW50Jyk7XG5Db21wbGV4LnByb3RvdHlwZS50b1N0cmluZyA9IHJlcXVpcmUoJy4vY29yZS9pbnN0YW5jZS90b1N0cmluZycpO1xuQ29tcGxleC5pc05hTiA9IHJlcXVpcmUoJy4vY29yZS9zdGF0aWMvaXNOYU4nKTtcbkNvbXBsZXguaXNFcXVhbCA9IHJlcXVpcmUoJy4vY29yZS9zdGF0aWMvaXNFcXVhbCcpO1xuQ29tcGxleC5jb25qdWdhdGUgPSByZXF1aXJlKCcuL2NvcmUvc3RhdGljL2Nvbmp1Z2F0ZScpO1xuQ29tcGxleC5pbnZlcnNlID0gcmVxdWlyZSgnLi9jb3JlL3N0YXRpYy9pbnZlcnNlJyk7XG5Db21wbGV4LmFkZCA9IHJlcXVpcmUoJy4vY29yZS9zdGF0aWMvYWRkJyk7XG5Db21wbGV4LnN1YnRyYWN0ID0gcmVxdWlyZSgnLi9jb3JlL3N0YXRpYy9zdWJ0cmFjdCcpO1xuQ29tcGxleC5tdWx0aXBseSA9IHJlcXVpcmUoJy4vY29yZS9zdGF0aWMvbXVsdGlwbHknKTtcbkNvbXBsZXguZGl2aWRlID0gcmVxdWlyZSgnLi9jb3JlL3N0YXRpYy9kaXZpZGUnKTtcbkNvbXBsZXguZXhwID0gcmVxdWlyZSgnLi9jb3JlL3N0YXRpYy9leHAnKTtcbkNvbXBsZXgubG9nID0gcmVxdWlyZSgnLi9jb3JlL3N0YXRpYy9sb2cnKTtcbkNvbXBsZXgucG93ID0gcmVxdWlyZSgnLi9jb3JlL3N0YXRpYy9wb3cnKTtcbkNvbXBsZXguc2luID0gcmVxdWlyZSgnLi9jb3JlL3N0YXRpYy9zaW4nKTtcbkNvbXBsZXguY29zID0gcmVxdWlyZSgnLi9jb3JlL3N0YXRpYy9jb3MnKTtcbkNvbXBsZXgudGFuID0gcmVxdWlyZSgnLi9jb3JlL3N0YXRpYy90YW4nKTtcbkNvbXBsZXguY3NjID0gcmVxdWlyZSgnLi9jb3JlL3N0YXRpYy9jc2MnKTtcbkNvbXBsZXguc2VjID0gcmVxdWlyZSgnLi9jb3JlL3N0YXRpYy9zZWMnKTtcbkNvbXBsZXguY290ID0gcmVxdWlyZSgnLi9jb3JlL3N0YXRpYy9jb3QnKTtcbkNvbXBsZXguYXNpbiA9IHJlcXVpcmUoJy4vY29yZS9zdGF0aWMvYXNpbicpO1xuQ29tcGxleC5hY29zID0gcmVxdWlyZSgnLi9jb3JlL3N0YXRpYy9hY29zJyk7XG5Db21wbGV4LmF0YW4gPSByZXF1aXJlKCcuL2NvcmUvc3RhdGljL2F0YW4nKTtcbkNvbXBsZXguYWNzYyA9IHJlcXVpcmUoJy4vY29yZS9zdGF0aWMvYWNzYycpO1xuQ29tcGxleC5hc2VjID0gcmVxdWlyZSgnLi9jb3JlL3N0YXRpYy9hc2VjJyk7XG5Db21wbGV4LmFjb3QgPSByZXF1aXJlKCcuL2NvcmUvc3RhdGljL2Fjb3QnKTtcbi8qKlxyXG4gKiBJdCByZXByZXNlbnRzIE5hTiBpbiB0aGlzIGxpYnJhcnkuIEl0IGlzIGVxdWl2YWxlbnQgdG8gbmV3IENvbXBsZXgoTmFOKS48YnI+PGJyPlxyXG4gKiBcclxuICogSXQgaXMgaW1wb3J0YW50IHRvIGtub3cgdGhhdCB0aGlzIGxpYnJhcnkgZG9lcyBub3QgaW50cm9kdWNlIHRoZSBjb25jZXB0IG9mIENvbXBsZXggSW5maW5pdHksXHJcbiAqIGFsbCBJbmZpbml0eSBpbiB0aGlzIGxpYnJhcnkgYXJlIHJlcHJlc2VudGVkIGJ5IENvbXBsZXguTmFOLlxyXG4gKiBAc3RhdGljXHJcbiAqL1xuXG5Db21wbGV4Lk5hTiA9IG5ldyBDb21wbGV4KE5hTik7XG4vKiogQHN0YXRpYyAqL1xuXG5Db21wbGV4Lk9ORSA9IG5ldyBDb21wbGV4KDEpO1xuLyoqIEBzdGF0aWMgKi9cblxuQ29tcGxleC5aRVJPID0gbmV3IENvbXBsZXgoMCk7XG4vKiogQHN0YXRpYyAqL1xuXG5Db21wbGV4LlBJID0gbmV3IENvbXBsZXgoTWF0aC5QSSk7XG4vKiogQHN0YXRpYyAqL1xuXG5Db21wbGV4LkUgPSBuZXcgQ29tcGxleChNYXRoLkUpO1xuLyoqXHJcbiAqIEl0IHJlcHJlc2VudHMgdGhlIHZhbHVlIG9mIDVlLTE2LCB3aGljaCBpcyB0aGUgc21hbGxlc3QgbnVtYmVyIGNvbnNpZGVyZWQgYXMgbm9uLXplcm8gaW4gdGhpcyBsaWJyYXJ5LlxyXG4gKiBJbiB0aGUgb3RoZXIgd29yZHMsIGFueSBudW1iZXIgbGVzcyB0aGFuIENvbXBsZXguRVBTSUxPTiBpcyBjb25zaWRlcmVkIGFzIDAuPGJyPjxicj5cclxuICogXHJcbiAqIE5vdGUgdGhhdCBDb21wbGV4LkVQU0lMT04gaXMgbnVtYmVyIGluc3RlYWQgb2YgaW5zdGFuY2Ugb2YgQ29tcGxleC5cclxuICogQHN0YXRpY1xyXG4gKi9cblxuQ29tcGxleC5FUFNJTE9OID0gMSAvIChNYXRoLnBvdygxMCwgMTUpICogMik7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbm1vZHVsZS5leHBvcnRzID0ge1xuICBJTlZBTElEX0FSUkFZOiAnSW52YWxpZCBhcmd1bWVudDogUmVjZWl2ZWQgYSBub24tYXJyYXkgYXJndW1lbnQnLFxuICBJTlZBTElEX01BVFJJWDogJ0ludmFsaWQgYXJndW1lbnQ6IFJlY2VpdmVkIGFuIGludmFsaWQgbWF0cml4JyxcbiAgSU5WQUxJRF9TUVVBUkVfTUFUUklYOiAnSW52YWxpZCBhcmd1bWVudDogUmVjZWl2ZWQgYSBub24tc3F1YXJlIG1hdHJpeCcsXG4gIElOVkFMSURfVVBQRVJfVFJJQU5HVUxBUl9NQVRSSVg6ICdJbnZhbGlkIGFyZ3VtZW50OiBSZWNlaXZlZCBhIG5vbiB1cHBlci10cmlhbmd1bGFyIG1hdHJpeCcsXG4gIElOVkFMSURfTE9XRVJfVFJJQU5HVUxBUl9NQVRSSVg6ICdJbnZhbGlkIGFyZ3VtZW50OiBSZWNlaXZlZCBhIG5vbiBsb3dlci10cmlhbmd1bGFyIG1hdHJpeCcsXG4gIElOVkFMSURfRVhQT05FTlQ6ICdJbnZhbGlkIGFyZ3VtZW50OiBFeHBlY3RlZCBhIG5vbi1uZWdhdGl2ZSBpbnRlZ2VyIGV4cG9uZW50JyxcbiAgSU5WQUxJRF9ST1dfQ09MOiAnSW52YWxpZCBhcmd1bWVudDogRXhwZWN0ZWQgbm9uLW5lZ2F0aXZlIGludGVnZXIgcm93IGFuZCBjb2x1bW4nLFxuICBJTlZBTElEX1JPVzogJ0ludmFsaWQgYXJndW1lbnQ6IEV4cGVjdGVkIG5vbi1uZWdhdGl2ZSBpbnRlZ2VyIHJvdycsXG4gIElOVkFMSURfQ09MVU1OOiAnSW52YWxpZCBhcmd1bWVudDogRXhwZWN0ZWQgbm9uLW5lZ2F0aXZlIGludGVnZXIgY29sdW1uJyxcbiAgSU5WQUxJRF9ST1dTX0VYUFJFU1NJT046ICdJbnZhbGlkIGFyZ3VtZW50OiBSZWNlaXZlZCBpbnZhbGlkIHJvd3MgZXhwcmVzc2lvbicsXG4gIElOVkFMSURfQ09MVU1OU19FWFBSRVNTSU9OOiAnSW52YWxpZCBhcmd1bWVudDogUmVjZWl2ZWQgaW52YWxpZCBjb2x1bW5zIGV4cHJlc3Npb24nLFxuICBJTlZBTElEX1BfTk9STTogJ0ludmFsaWQgYXJndW1lbnQ6IFJlY2VpdmVkIGludmFsaWQgcC1ub3JtJyxcbiAgT1ZFUkZMT1dfSU5ERVg6ICdJbnZhbGlkIGFyZ3VtZW50OiBNYXRyaXggaW5kZXggb3ZlcmZsb3cnLFxuICBPVkVSRkxPV19DT0xVTU46ICdJbnZhbGlkIGFyZ3VtZW50OiBDb2x1bW4gaW5kZXggb3ZlcmZsb3cnLFxuICBPVkVSRkxPV19ST1c6ICdJbnZhbGlkIGFyZ3VtZW50OiBSb3cgaW5kZXggb3ZlcmZsb3cnLFxuICBOT19VTklRVUVfU09MVVRJT046ICdBcml0aG1ldGljIEV4Y2VwdGlvbjogVGhlIHN5c3RlbSBoYXMgbm8gdW5pcXVlIHNvbHV0aW9uJyxcbiAgU0laRV9JTkNPTVBBVElCTEU6ICdJbnZhbGlkIGFyZ3VtZW50OiBNYXRyaXggc2l6ZS1pbmNvbXBhdGlibGUnLFxuICBTSU5HVUxBUl9NQVRSSVg6ICdBcml0aG1ldGljIEV4Y2VwdGlvbjogVGhlIG1hdHJpeCBpcyBub3QgaW52ZXJ0aWJsZScsXG4gIEVYUEVDVEVEX1NUUklOR19OVU1CRVJfQVRfUE9TXzFfMjogJ0ludmFsaWQgYXJndW1lbnQ6IEV4cGVjdGVkIGEgc3RyaW5nIG9yIGEgbnVtYmVyIGF0IGFyZ3VtZW50c1sxXSBhbmQgYXJndW1lbnRzWzJdJyxcbiAgRVhQRUNURURfQVJSQVlfT0ZfTlVNQkVSU19PUl9NQVRSSUNFUzogJ0ludmFsaWQgYXJndW1lbnQ6IEV4cGVjdGVkIGVpdGhlciBhbiBhcnJheSBvZiBudW1iZXJzIG9yIGFuIGFycmF5IG9mIHNxdWFyZSBtYXRyaWNlcydcbn07IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbmZ1bmN0aW9uIF9zbGljZWRUb0FycmF5KGFyciwgaSkgeyByZXR1cm4gX2FycmF5V2l0aEhvbGVzKGFycikgfHwgX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgfHwgX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KGFyciwgaSkgfHwgX25vbkl0ZXJhYmxlUmVzdCgpOyB9XG5cbmZ1bmN0aW9uIF9ub25JdGVyYWJsZVJlc3QoKSB7IHRocm93IG5ldyBUeXBlRXJyb3IoXCJJbnZhbGlkIGF0dGVtcHQgdG8gZGVzdHJ1Y3R1cmUgbm9uLWl0ZXJhYmxlIGluc3RhbmNlLlxcbkluIG9yZGVyIHRvIGJlIGl0ZXJhYmxlLCBub24tYXJyYXkgb2JqZWN0cyBtdXN0IGhhdmUgYSBbU3ltYm9sLml0ZXJhdG9yXSgpIG1ldGhvZC5cIik7IH1cblxuZnVuY3Rpb24gX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KG8sIG1pbkxlbikgeyBpZiAoIW8pIHJldHVybjsgaWYgKHR5cGVvZiBvID09PSBcInN0cmluZ1wiKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgdmFyIG4gPSBPYmplY3QucHJvdG90eXBlLnRvU3RyaW5nLmNhbGwobykuc2xpY2UoOCwgLTEpOyBpZiAobiA9PT0gXCJPYmplY3RcIiAmJiBvLmNvbnN0cnVjdG9yKSBuID0gby5jb25zdHJ1Y3Rvci5uYW1lOyBpZiAobiA9PT0gXCJNYXBcIiB8fCBuID09PSBcIlNldFwiKSByZXR1cm4gQXJyYXkuZnJvbShvKTsgaWYgKG4gPT09IFwiQXJndW1lbnRzXCIgfHwgL14oPzpVaXxJKW50KD86OHwxNnwzMikoPzpDbGFtcGVkKT9BcnJheSQvLnRlc3QobikpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB9XG5cbmZ1bmN0aW9uIF9hcnJheUxpa2VUb0FycmF5KGFyciwgbGVuKSB7IGlmIChsZW4gPT0gbnVsbCB8fCBsZW4gPiBhcnIubGVuZ3RoKSBsZW4gPSBhcnIubGVuZ3RoOyBmb3IgKHZhciBpID0gMCwgYXJyMiA9IG5ldyBBcnJheShsZW4pOyBpIDwgbGVuOyBpKyspIHsgYXJyMltpXSA9IGFycltpXTsgfSByZXR1cm4gYXJyMjsgfVxuXG5mdW5jdGlvbiBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB7IGlmICh0eXBlb2YgU3ltYm9sID09PSBcInVuZGVmaW5lZFwiIHx8ICEoU3ltYm9sLml0ZXJhdG9yIGluIE9iamVjdChhcnIpKSkgcmV0dXJuOyB2YXIgX2FyciA9IFtdOyB2YXIgX24gPSB0cnVlOyB2YXIgX2QgPSBmYWxzZTsgdmFyIF9lID0gdW5kZWZpbmVkOyB0cnkgeyBmb3IgKHZhciBfaSA9IGFycltTeW1ib2wuaXRlcmF0b3JdKCksIF9zOyAhKF9uID0gKF9zID0gX2kubmV4dCgpKS5kb25lKTsgX24gPSB0cnVlKSB7IF9hcnIucHVzaChfcy52YWx1ZSk7IGlmIChpICYmIF9hcnIubGVuZ3RoID09PSBpKSBicmVhazsgfSB9IGNhdGNoIChlcnIpIHsgX2QgPSB0cnVlOyBfZSA9IGVycjsgfSBmaW5hbGx5IHsgdHJ5IHsgaWYgKCFfbiAmJiBfaVtcInJldHVyblwiXSAhPSBudWxsKSBfaVtcInJldHVyblwiXSgpOyB9IGZpbmFsbHkgeyBpZiAoX2QpIHRocm93IF9lOyB9IH0gcmV0dXJuIF9hcnI7IH1cblxuZnVuY3Rpb24gX2FycmF5V2l0aEhvbGVzKGFycikgeyBpZiAoQXJyYXkuaXNBcnJheShhcnIpKSByZXR1cm4gYXJyOyB9XG5cbnZhciBfcmVxdWlyZSA9IHJlcXVpcmUoJy4uLy4uL0Vycm9yJyksXG4gICAgSU5WQUxJRF9NQVRSSVggPSBfcmVxdWlyZS5JTlZBTElEX01BVFJJWDtcbi8qKlxyXG4gKiBDYWxjdWxhdGVzIHRoZSBMVVAgZGVjb21wb3NpdGlvbiBvZiB0aGUgTWF0cml4LFxyXG4gKiB3aGVyZSBMIGlzIGxvd2VyIHRyaWFuZ3VsYXIgbWF0cml4IHdoaWNoIGRpYWdvbmFsIGVudHJpZXMgYXJlIGFsd2F5cyAxLFxyXG4gKiBVIGlzIHVwcGVyIHRyaWFuZ3VsYXIgbWF0cml4LCBhbmQgUCBpcyBwZXJtdXRhdGlvbiBtYXRyaXguPGJyPjxicj5cclxuICogXHJcbiAqIEl0IGlzIGltcGxlbWVudGVkIHVzaW5nIEdhdXNzaWFuIEVsaW1pbmF0aW9uIHdpdGggUGFydGlhbCBQaXZvdGluZyBpbiBvcmRlciB0b1xyXG4gKiByZWR1Y2UgdGhlIGVycm9yIGNhdXNlZCBieSBmbG9hdGluZy1wb2ludCBhcml0aG1ldGljLjxicj48YnI+XHJcbiAqIFxyXG4gKiBOb3RlIHRoYXQgaWYgb3B0aW1pemVkIGlzIHRydWUsIFAgaXMgYSBQZXJtdXRhdGlvbiBBcnJheSBhbmQgYm90aCBMIGFuZCBVIGFyZSBtZXJnZWRcclxuICogaW50byBvbmUgbWF0cml4IGluIG9yZGVyIHRvIGltcHJvdmUgcGVyZm9ybWFuY2UuXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge01hdHJpeH0gQSAtIEFueSBtYXRyaXhcclxuICogQHBhcmFtIHtib29sZWFufSBbb3B0aW1pemVkPWZhbHNlXSAtIFJldHVybnMgW1AsIExVXSBpZiBpdCBpcyB0cnVlLCBbUCwgTCwgVV0gaWYgaXQgaXMgZmFsc2VcclxuICogQHJldHVybnMge01hdHJpeFtdfSBUaGUgTFVQIGRlY29tcG9zaXRpb24gb2YgTWF0cml4XHJcbiAqL1xuXG5cbmZ1bmN0aW9uIExVKEEpIHtcbiAgdmFyIG9wdGltaXplZCA9IGFyZ3VtZW50cy5sZW5ndGggPiAxICYmIGFyZ3VtZW50c1sxXSAhPT0gdW5kZWZpbmVkID8gYXJndW1lbnRzWzFdIDogZmFsc2U7XG5cbiAgaWYgKCEoQSBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfTUFUUklYKTtcbiAgfVxuXG4gIHZhciBfQSRzaXplID0gQS5zaXplKCksXG4gICAgICBfQSRzaXplMiA9IF9zbGljZWRUb0FycmF5KF9BJHNpemUsIDIpLFxuICAgICAgcm93ID0gX0Ekc2l6ZTJbMF0sXG4gICAgICBjb2wgPSBfQSRzaXplMlsxXTtcblxuICB2YXIgc2l6ZSA9IE1hdGgubWluKHJvdywgY29sKTtcbiAgdmFyIEVQU0lMT04gPSAxIC8gKE1hdGgucG93KDEwLCBBLl9kaWdpdCkgKiAyKTtcbiAgdmFyIHBlcm11dGF0aW9uID0gaW5pdFBlcm11dGF0aW9uKHJvdyk7XG5cbiAgdmFyIGNvcHkgPSB0aGlzLmNsb25lKEEpLl9tYXRyaXg7XG5cbiAgZm9yICh2YXIgaSA9IDA7IGkgPCByb3cgLSAxOyBpKyspIHtcbiAgICB2YXIgY3VycmVudENvbCA9IE1hdGgubWluKGksIGNvbCk7IC8vIGFwcGx5IFBhcnRpYWwgUGl2b3RpbmdcblxuICAgIFBhcnRpYWxQaXZvdGluZyhjb3B5LCBwZXJtdXRhdGlvbiwgY3VycmVudENvbCwgcm93LCBjb2wpO1xuICAgIHZhciBpdGggPSBwZXJtdXRhdGlvbltpXTtcbiAgICB2YXIgcGl2b3QgPSBjb3B5W2l0aF1bY3VycmVudENvbF07XG5cbiAgICBpZiAoTWF0aC5hYnMocGl2b3QpIDwgRVBTSUxPTikge1xuICAgICAgY29udGludWU7XG4gICAgfVxuXG4gICAgZm9yICh2YXIgaiA9IGkgKyAxOyBqIDwgcm93OyBqKyspIHtcbiAgICAgIHZhciBqdGggPSBwZXJtdXRhdGlvbltqXTtcbiAgICAgIHZhciBlbnRyeSA9IGNvcHlbanRoXVtjdXJyZW50Q29sXTtcblxuICAgICAgaWYgKE1hdGguYWJzKGVudHJ5KSA+PSBFUFNJTE9OKSB7XG4gICAgICAgIHZhciBmYWN0b3IgPSBlbnRyeSAvIHBpdm90O1xuXG4gICAgICAgIGZvciAodmFyIGsgPSBjdXJyZW50Q29sOyBrIDwgY29sOyBrKyspIHtcbiAgICAgICAgICBjb3B5W2p0aF1ba10gLT0gZmFjdG9yICogY29weVtpdGhdW2tdO1xuICAgICAgICB9XG5cbiAgICAgICAgY29weVtqdGhdW2N1cnJlbnRDb2xdID0gZmFjdG9yO1xuICAgICAgfVxuICAgIH1cbiAgfVxuXG4gIHZhciByZXN1bHQgPSBuZXcgQXJyYXkocm93KTtcblxuICBmb3IgKHZhciBfaTIgPSAwOyBfaTIgPCByb3c7IF9pMisrKSB7XG4gICAgcmVzdWx0W19pMl0gPSBjb3B5W3Blcm11dGF0aW9uW19pMl1dO1xuICB9XG5cbiAgaWYgKG9wdGltaXplZCkge1xuICAgIHJldHVybiBbcGVybXV0YXRpb24sIG5ldyB0aGlzKHJlc3VsdCldO1xuICB9XG5cbiAgdmFyIFAgPSB0aGlzLmdlbmVyYXRlKHJvdywgcm93LCBmdW5jdGlvbiAoaSwgaikge1xuICAgIHZhciBpZHggPSBwZXJtdXRhdGlvbltpXTtcblxuICAgIGlmIChqID09PSBpZHgpIHtcbiAgICAgIHJldHVybiAxO1xuICAgIH1cblxuICAgIHJldHVybiAwO1xuICB9KTtcbiAgdmFyIEwgPSB0aGlzLmdlbmVyYXRlKHJvdywgc2l6ZSwgZnVuY3Rpb24gKGksIGopIHtcbiAgICBpZiAoaSA9PT0gaikge1xuICAgICAgcmV0dXJuIDE7XG4gICAgfVxuXG4gICAgaWYgKGkgPCBqKSB7XG4gICAgICByZXR1cm4gMDtcbiAgICB9XG5cbiAgICByZXR1cm4gcmVzdWx0W2ldW2pdO1xuICB9KTtcbiAgdmFyIFUgPSB0aGlzLmdlbmVyYXRlKHNpemUsIGNvbCwgZnVuY3Rpb24gKGksIGopIHtcbiAgICBpZiAoaSA+IGopIHtcbiAgICAgIHJldHVybiAwO1xuICAgIH1cblxuICAgIHJldHVybiByZXN1bHRbaV1bal07XG4gIH0pO1xuICByZXR1cm4gW1AsIEwsIFVdO1xufVxuXG47XG5cbmZ1bmN0aW9uIGluaXRQZXJtdXRhdGlvbihzaXplKSB7XG4gIHZhciBwZXJtdXRhdGlvbiA9IG5ldyBBcnJheShzaXplKTtcblxuICBmb3IgKHZhciBpID0gMDsgaSA8IHNpemU7IGkrKykge1xuICAgIHBlcm11dGF0aW9uW2ldID0gaTtcbiAgfVxuXG4gIHJldHVybiBwZXJtdXRhdGlvbjtcbn1cblxuZnVuY3Rpb24gUGFydGlhbFBpdm90aW5nKG1hdHJpeCwgcGVybXV0YXRpb24sIHBvcywgcm93LCBjb2wpIHtcbiAgdmFyIGN1cnJlbnRDb2wgPSBNYXRoLm1pbihwb3MsIGNvbCk7XG4gIHZhciBtYXhJZHggPSBwb3M7XG4gIHZhciBtYXggPSBNYXRoLmFicyhtYXRyaXhbcGVybXV0YXRpb25bcG9zXV1bY3VycmVudENvbF0pO1xuXG4gIGZvciAodmFyIGkgPSBwb3MgKyAxOyBpIDwgcm93OyBpKyspIHtcbiAgICB2YXIgdmFsdWUgPSBNYXRoLmFicyhtYXRyaXhbcGVybXV0YXRpb25baV1dW2N1cnJlbnRDb2xdKTtcblxuICAgIGlmICh2YWx1ZSA+IG1heCkge1xuICAgICAgbWF4SWR4ID0gaTtcbiAgICAgIG1heCA9IHZhbHVlO1xuICAgIH1cbiAgfVxuXG4gIHZhciB0ID0gcGVybXV0YXRpb25bcG9zXTtcbiAgcGVybXV0YXRpb25bcG9zXSA9IHBlcm11dGF0aW9uW21heElkeF07XG4gIHBlcm11dGF0aW9uW21heElkeF0gPSB0O1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IExVOyIsIlwidXNlIHN0cmljdFwiO1xuXG5mdW5jdGlvbiBfc2xpY2VkVG9BcnJheShhcnIsIGkpIHsgcmV0dXJuIF9hcnJheVdpdGhIb2xlcyhhcnIpIHx8IF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHx8IF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShhcnIsIGkpIHx8IF9ub25JdGVyYWJsZVJlc3QoKTsgfVxuXG5mdW5jdGlvbiBfbm9uSXRlcmFibGVSZXN0KCkgeyB0aHJvdyBuZXcgVHlwZUVycm9yKFwiSW52YWxpZCBhdHRlbXB0IHRvIGRlc3RydWN0dXJlIG5vbi1pdGVyYWJsZSBpbnN0YW5jZS5cXG5JbiBvcmRlciB0byBiZSBpdGVyYWJsZSwgbm9uLWFycmF5IG9iamVjdHMgbXVzdCBoYXZlIGEgW1N5bWJvbC5pdGVyYXRvcl0oKSBtZXRob2QuXCIpOyB9XG5cbmZ1bmN0aW9uIF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShvLCBtaW5MZW4pIHsgaWYgKCFvKSByZXR1cm47IGlmICh0eXBlb2YgbyA9PT0gXCJzdHJpbmdcIikgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IHZhciBuID0gT2JqZWN0LnByb3RvdHlwZS50b1N0cmluZy5jYWxsKG8pLnNsaWNlKDgsIC0xKTsgaWYgKG4gPT09IFwiT2JqZWN0XCIgJiYgby5jb25zdHJ1Y3RvcikgbiA9IG8uY29uc3RydWN0b3IubmFtZTsgaWYgKG4gPT09IFwiTWFwXCIgfHwgbiA9PT0gXCJTZXRcIikgcmV0dXJuIEFycmF5LmZyb20obyk7IGlmIChuID09PSBcIkFyZ3VtZW50c1wiIHx8IC9eKD86VWl8SSludCg/Ojh8MTZ8MzIpKD86Q2xhbXBlZCk/QXJyYXkkLy50ZXN0KG4pKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgfVxuXG5mdW5jdGlvbiBfYXJyYXlMaWtlVG9BcnJheShhcnIsIGxlbikgeyBpZiAobGVuID09IG51bGwgfHwgbGVuID4gYXJyLmxlbmd0aCkgbGVuID0gYXJyLmxlbmd0aDsgZm9yICh2YXIgaSA9IDAsIGFycjIgPSBuZXcgQXJyYXkobGVuKTsgaSA8IGxlbjsgaSsrKSB7IGFycjJbaV0gPSBhcnJbaV07IH0gcmV0dXJuIGFycjI7IH1cblxuZnVuY3Rpb24gX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgeyBpZiAodHlwZW9mIFN5bWJvbCA9PT0gXCJ1bmRlZmluZWRcIiB8fCAhKFN5bWJvbC5pdGVyYXRvciBpbiBPYmplY3QoYXJyKSkpIHJldHVybjsgdmFyIF9hcnIgPSBbXTsgdmFyIF9uID0gdHJ1ZTsgdmFyIF9kID0gZmFsc2U7IHZhciBfZSA9IHVuZGVmaW5lZDsgdHJ5IHsgZm9yICh2YXIgX2kgPSBhcnJbU3ltYm9sLml0ZXJhdG9yXSgpLCBfczsgIShfbiA9IChfcyA9IF9pLm5leHQoKSkuZG9uZSk7IF9uID0gdHJ1ZSkgeyBfYXJyLnB1c2goX3MudmFsdWUpOyBpZiAoaSAmJiBfYXJyLmxlbmd0aCA9PT0gaSkgYnJlYWs7IH0gfSBjYXRjaCAoZXJyKSB7IF9kID0gdHJ1ZTsgX2UgPSBlcnI7IH0gZmluYWxseSB7IHRyeSB7IGlmICghX24gJiYgX2lbXCJyZXR1cm5cIl0gIT0gbnVsbCkgX2lbXCJyZXR1cm5cIl0oKTsgfSBmaW5hbGx5IHsgaWYgKF9kKSB0aHJvdyBfZTsgfSB9IHJldHVybiBfYXJyOyB9XG5cbmZ1bmN0aW9uIF9hcnJheVdpdGhIb2xlcyhhcnIpIHsgaWYgKEFycmF5LmlzQXJyYXkoYXJyKSkgcmV0dXJuIGFycjsgfVxuXG52YXIgX3JlcXVpcmUgPSByZXF1aXJlKCcuLi8uLi9FcnJvcicpLFxuICAgIElOVkFMSURfTUFUUklYID0gX3JlcXVpcmUuSU5WQUxJRF9NQVRSSVg7XG4vKipcclxuICogQ2FsY3VsYXRlcyB0aGUgUVIgZGVjb21wb3NpdGlvbiBvZiB0aGUgTWF0cml4XHJcbiAqIHdoZXJlIFEgaXMgb3J0aG9nb25hbCBtYXRyaXgsIFIgaXMgdXBwZXIgdHJpYW5ndWxhciBtYXRyaXguPGJyPjxicj5cclxuICogXHJcbiAqIFRoZSBhbGdvcml0aG0gaXMgaW1wbGVtZW50ZWQgdXNpbmcgSG91c2Vob2xkZXIgVHJhbnNmb3JtIGluc3RlYWQgb2YgR3JhbeKAk1NjaG1pZHQgcHJvY2Vzc1xyXG4gKiBiZWNhdXNlIHRoZSBIb3VzZWhvbGRlciBUcmFuc2Zvcm0gaXMgbW9yZSBudW1lcmljYWxseSBzdGFibGUuXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge01hdHJpeH0gQSAtIEFueSBtYXRyaXhcclxuICogQHJldHVybnMge01hdHJpeFtdfSBUaGUgUVIgZGVjb21wb3NpdGlvbiBvZiBtYXRyaXggaW4gdGhlIGZvcm0gb2YgW1EsIFJdXHJcbiAqL1xuXG5cbmZ1bmN0aW9uIFFSKEEpIHtcbiAgaWYgKCEoQSBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfTUFUUklYKTtcbiAgfVxuXG4gIHZhciBfQSRzaXplID0gQS5zaXplKCksXG4gICAgICBfQSRzaXplMiA9IF9zbGljZWRUb0FycmF5KF9BJHNpemUsIDIpLFxuICAgICAgcm93ID0gX0Ekc2l6ZTJbMF0sXG4gICAgICBjb2wgPSBfQSRzaXplMlsxXTtcblxuICB2YXIgc2l6ZSA9IE1hdGgubWluKHJvdywgY29sKTtcbiAgdmFyIEVQU0lMT04gPSAxIC8gKE1hdGgucG93KDEwLCBBLl9kaWdpdCkgKiAyKTtcblxuICB2YXIgbWF0cml4UiA9IHRoaXMuY2xvbmUoQSkuX21hdHJpeDtcblxuICB2YXIgbWF0cml4USA9IHRoaXMuaWRlbnRpdHkocm93KS5fbWF0cml4O1xuXG4gIGZvciAodmFyIGogPSAwOyBqIDwgc2l6ZTsgaisrKSB7XG4gICAgLy8gaWYgYWxsIGVudHJpZXMgYmVsb3cgbWFpbiBkaWFnb25hbCBhcmUgY29uc2lkZXJlZCBhcyB6ZXJvLCBza2lwIHRoaXMgcm91bmRcbiAgICB2YXIgc2tpcCA9IHRydWU7XG5cbiAgICBmb3IgKHZhciBpID0gaiArIDE7IGkgPCByb3c7IGkrKykge1xuICAgICAgaWYgKE1hdGguYWJzKG1hdHJpeFJbaV1bal0pID49IEVQU0lMT04pIHtcbiAgICAgICAgc2tpcCA9IGZhbHNlO1xuICAgICAgICBicmVhaztcbiAgICAgIH1cbiAgICB9XG5cbiAgICBpZiAoIXNraXApIHtcbiAgICAgIC8vIEFwcGx5IEhvdXNlaG9sZGVyIHRyYW5zZm9ybVxuICAgICAgdmFyIG5vcm0gPSAwO1xuXG4gICAgICBmb3IgKHZhciBfaTIgPSBqOyBfaTIgPCByb3c7IF9pMisrKSB7XG4gICAgICAgIG5vcm0gKz0gTWF0aC5wb3cobWF0cml4UltfaTJdW2pdLCAyKTtcbiAgICAgIH1cblxuICAgICAgbm9ybSA9IE1hdGguc3FydChub3JtKTsgLy8gcmVkdWNlIGZsb2F0aW5nIHBvaW50IGFyaXRobWF0aWMgZXJyb3JcblxuICAgICAgdmFyIHMgPSAtMTtcblxuICAgICAgaWYgKG1hdHJpeFJbal1bal0gPCAwKSB7XG4gICAgICAgIHMgPSAxO1xuICAgICAgfVxuXG4gICAgICB2YXIgdTEgPSBtYXRyaXhSW2pdW2pdIC0gcyAqIG5vcm07XG4gICAgICB2YXIgdyA9IG5ldyBBcnJheShyb3cgLSBqKTtcblxuICAgICAgZm9yICh2YXIgX2kzID0gMDsgX2kzIDwgcm93IC0gajsgX2kzKyspIHtcbiAgICAgICAgd1tfaTNdID0gbWF0cml4UltfaTMgKyBqXVtqXSAvIHUxO1xuICAgICAgfVxuXG4gICAgICB3WzBdID0gMTtcbiAgICAgIHZhciB0YXUgPSAtMSAqIHMgKiB1MSAvIG5vcm07XG4gICAgICB2YXIgc3ViUiA9IG5ldyBBcnJheShyb3cgLSBqKTtcblxuICAgICAgZm9yICh2YXIgX2k0ID0gMDsgX2k0IDwgcm93IC0gajsgX2k0KyspIHtcbiAgICAgICAgdmFyIG5ld1JvdyA9IG5ldyBBcnJheShjb2wpO1xuXG4gICAgICAgIGZvciAodmFyIGsgPSAwOyBrIDwgY29sOyBrKyspIHtcbiAgICAgICAgICBuZXdSb3dba10gPSBtYXRyaXhSW2ogKyBfaTRdW2tdO1xuICAgICAgICB9XG5cbiAgICAgICAgc3ViUltfaTRdID0gbmV3Um93O1xuICAgICAgfVxuXG4gICAgICBmb3IgKHZhciBfaTUgPSBqOyBfaTUgPCByb3c7IF9pNSsrKSB7XG4gICAgICAgIGZvciAodmFyIF9rID0gMDsgX2sgPCBjb2w7IF9rKyspIHtcbiAgICAgICAgICB2YXIgc3VtbWF0aW9uID0gMDtcblxuICAgICAgICAgIGZvciAodmFyIG0gPSAwOyBtIDwgcm93IC0gajsgbSsrKSB7XG4gICAgICAgICAgICBzdW1tYXRpb24gKz0gc3ViUlttXVtfa10gKiB3W21dO1xuICAgICAgICAgIH1cblxuICAgICAgICAgIG1hdHJpeFJbX2k1XVtfa10gPSBzdWJSW19pNSAtIGpdW19rXSAtIHRhdSAqIHdbX2k1IC0gal0gKiBzdW1tYXRpb247XG4gICAgICAgIH1cbiAgICAgIH1cblxuICAgICAgdmFyIHN1YlEgPSBuZXcgQXJyYXkocm93KTtcblxuICAgICAgZm9yICh2YXIgX2k2ID0gMDsgX2k2IDwgcm93OyBfaTYrKykge1xuICAgICAgICB2YXIgX25ld1JvdyA9IG5ldyBBcnJheShyb3cgLSBqKTtcblxuICAgICAgICBmb3IgKHZhciBfazIgPSAwOyBfazIgPCByb3cgLSBqOyBfazIrKykge1xuICAgICAgICAgIF9uZXdSb3dbX2syXSA9IG1hdHJpeFFbX2k2XVtqICsgX2syXTtcbiAgICAgICAgfVxuXG4gICAgICAgIHN1YlFbX2k2XSA9IF9uZXdSb3c7XG4gICAgICB9XG5cbiAgICAgIGZvciAodmFyIF9pNyA9IDA7IF9pNyA8IHJvdzsgX2k3KyspIHtcbiAgICAgICAgZm9yICh2YXIgX2szID0gajsgX2szIDwgcm93OyBfazMrKykge1xuICAgICAgICAgIHZhciBfc3VtbWF0aW9uID0gMDtcblxuICAgICAgICAgIGZvciAodmFyIF9tID0gMDsgX20gPCByb3cgLSBqOyBfbSsrKSB7XG4gICAgICAgICAgICBfc3VtbWF0aW9uICs9IHN1YlFbX2k3XVtfbV0gKiB3W19tXTtcbiAgICAgICAgICB9XG5cbiAgICAgICAgICBtYXRyaXhRW19pN11bX2szXSA9IHN1YlFbX2k3XVtfazMgLSBqXSAtIHRhdSAqIHdbX2szIC0gal0gKiBfc3VtbWF0aW9uO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuICB9XG5cbiAgZm9yICh2YXIgX2k4ID0gMDsgX2k4IDwgcm93OyBfaTgrKykge1xuICAgIGZvciAodmFyIF9qID0gMDsgX2ogPCBjb2w7IF9qKyspIHtcbiAgICAgIGlmIChfaTggPiBfaikge1xuICAgICAgICBtYXRyaXhSW19pOF1bX2pdID0gMDtcbiAgICAgIH1cbiAgICB9XG4gIH1cblxuICByZXR1cm4gW25ldyB0aGlzKG1hdHJpeFEpLCBuZXcgdGhpcyhtYXRyaXhSKV07XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gUVI7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbmZ1bmN0aW9uIF9zbGljZWRUb0FycmF5KGFyciwgaSkgeyByZXR1cm4gX2FycmF5V2l0aEhvbGVzKGFycikgfHwgX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgfHwgX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KGFyciwgaSkgfHwgX25vbkl0ZXJhYmxlUmVzdCgpOyB9XG5cbmZ1bmN0aW9uIF9ub25JdGVyYWJsZVJlc3QoKSB7IHRocm93IG5ldyBUeXBlRXJyb3IoXCJJbnZhbGlkIGF0dGVtcHQgdG8gZGVzdHJ1Y3R1cmUgbm9uLWl0ZXJhYmxlIGluc3RhbmNlLlxcbkluIG9yZGVyIHRvIGJlIGl0ZXJhYmxlLCBub24tYXJyYXkgb2JqZWN0cyBtdXN0IGhhdmUgYSBbU3ltYm9sLml0ZXJhdG9yXSgpIG1ldGhvZC5cIik7IH1cblxuZnVuY3Rpb24gX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KG8sIG1pbkxlbikgeyBpZiAoIW8pIHJldHVybjsgaWYgKHR5cGVvZiBvID09PSBcInN0cmluZ1wiKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgdmFyIG4gPSBPYmplY3QucHJvdG90eXBlLnRvU3RyaW5nLmNhbGwobykuc2xpY2UoOCwgLTEpOyBpZiAobiA9PT0gXCJPYmplY3RcIiAmJiBvLmNvbnN0cnVjdG9yKSBuID0gby5jb25zdHJ1Y3Rvci5uYW1lOyBpZiAobiA9PT0gXCJNYXBcIiB8fCBuID09PSBcIlNldFwiKSByZXR1cm4gQXJyYXkuZnJvbShvKTsgaWYgKG4gPT09IFwiQXJndW1lbnRzXCIgfHwgL14oPzpVaXxJKW50KD86OHwxNnwzMikoPzpDbGFtcGVkKT9BcnJheSQvLnRlc3QobikpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB9XG5cbmZ1bmN0aW9uIF9hcnJheUxpa2VUb0FycmF5KGFyciwgbGVuKSB7IGlmIChsZW4gPT0gbnVsbCB8fCBsZW4gPiBhcnIubGVuZ3RoKSBsZW4gPSBhcnIubGVuZ3RoOyBmb3IgKHZhciBpID0gMCwgYXJyMiA9IG5ldyBBcnJheShsZW4pOyBpIDwgbGVuOyBpKyspIHsgYXJyMltpXSA9IGFycltpXTsgfSByZXR1cm4gYXJyMjsgfVxuXG5mdW5jdGlvbiBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB7IGlmICh0eXBlb2YgU3ltYm9sID09PSBcInVuZGVmaW5lZFwiIHx8ICEoU3ltYm9sLml0ZXJhdG9yIGluIE9iamVjdChhcnIpKSkgcmV0dXJuOyB2YXIgX2FyciA9IFtdOyB2YXIgX24gPSB0cnVlOyB2YXIgX2QgPSBmYWxzZTsgdmFyIF9lID0gdW5kZWZpbmVkOyB0cnkgeyBmb3IgKHZhciBfaSA9IGFycltTeW1ib2wuaXRlcmF0b3JdKCksIF9zOyAhKF9uID0gKF9zID0gX2kubmV4dCgpKS5kb25lKTsgX24gPSB0cnVlKSB7IF9hcnIucHVzaChfcy52YWx1ZSk7IGlmIChpICYmIF9hcnIubGVuZ3RoID09PSBpKSBicmVhazsgfSB9IGNhdGNoIChlcnIpIHsgX2QgPSB0cnVlOyBfZSA9IGVycjsgfSBmaW5hbGx5IHsgdHJ5IHsgaWYgKCFfbiAmJiBfaVtcInJldHVyblwiXSAhPSBudWxsKSBfaVtcInJldHVyblwiXSgpOyB9IGZpbmFsbHkgeyBpZiAoX2QpIHRocm93IF9lOyB9IH0gcmV0dXJuIF9hcnI7IH1cblxuZnVuY3Rpb24gX2FycmF5V2l0aEhvbGVzKGFycikgeyBpZiAoQXJyYXkuaXNBcnJheShhcnIpKSByZXR1cm4gYXJyOyB9XG5cbnZhciBlbXB0eSA9IHJlcXVpcmUoJy4uLy4uL3V0aWwvZW1wdHknKTtcblxudmFyIF9yZXF1aXJlID0gcmVxdWlyZSgnLi4vLi4vRXJyb3InKSxcbiAgICBJTlZBTElEX01BVFJJWCA9IF9yZXF1aXJlLklOVkFMSURfTUFUUklYLFxuICAgIElOVkFMSURfVVBQRVJfVFJJQU5HVUxBUl9NQVRSSVggPSBfcmVxdWlyZS5JTlZBTElEX1VQUEVSX1RSSUFOR1VMQVJfTUFUUklYLFxuICAgIElOVkFMSURfU1FVQVJFX01BVFJJWCA9IF9yZXF1aXJlLklOVkFMSURfU1FVQVJFX01BVFJJWCxcbiAgICBTSVpFX0lOQ09NUEFUSUJMRSA9IF9yZXF1aXJlLlNJWkVfSU5DT01QQVRJQkxFLFxuICAgIE5PX1VOSVFVRV9TT0xVVElPTiA9IF9yZXF1aXJlLk5PX1VOSVFVRV9TT0xVVElPTjtcbi8qKlxyXG4qIFNvbHZlIHN5c3RlbSBvZiBsaW5lYXIgZXF1YXRpb25zIFV4ID0geSB1c2luZyBiYWNrd2FyZCBzdWJzdGl0dXRpb24sXHJcbiogd2hlcmUgVSBpcyBhbiB1cHBlciB0cmlhbmd1bGFyIG1hdHJpeC5cclxuKiBJZiB0aGVyZSBpcyBubyB1bmlxdWUgc29sdXRpb25zLCBhbiBlcnJvciBpcyB0aHJvd24uXHJcbiogQG1lbWJlcm9mIE1hdHJpeFxyXG4qIEBzdGF0aWNcclxuKiBAcGFyYW0ge01hdHJpeH0gVSAtIEFueSBuIHggbiB1cHBlciB0cmlhbmd1bGFyIE1hdHJpeFxyXG4qIEBwYXJhbSB7TWF0cml4fSB5IC0gQW55IG4geCAxIE1hdHJpeFxyXG4qIEByZXR1cm5zIHtNYXRyaXh9IG4geCAxIE1hdHJpeCB3aGljaCBpcyB0aGUgc29sdXRpb24gb2YgVXggPSB5XHJcbiovXG5cblxuZnVuY3Rpb24gYmFja3dhcmQoVSwgeSkge1xuICBpZiAoIShVIGluc3RhbmNlb2YgdGhpcykgfHwgISh5IGluc3RhbmNlb2YgdGhpcykpIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoSU5WQUxJRF9NQVRSSVgpO1xuICB9XG5cbiAgaWYgKCFVLmlzVXBwZXJUcmlhbmd1bGFyKCkpIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoSU5WQUxJRF9VUFBFUl9UUklBTkdVTEFSX01BVFJJWCk7XG4gIH1cblxuICBpZiAoIVUuaXNTcXVhcmUoKSkge1xuICAgIHRocm93IG5ldyBFcnJvcihJTlZBTElEX1NRVUFSRV9NQVRSSVgpO1xuICB9XG5cbiAgdmFyIHNpemUgPSBVLnNpemUoKVswXTtcblxuICB2YXIgX3kkc2l6ZSA9IHkuc2l6ZSgpLFxuICAgICAgX3kkc2l6ZTIgPSBfc2xpY2VkVG9BcnJheShfeSRzaXplLCAyKSxcbiAgICAgIHlyb3cgPSBfeSRzaXplMlswXSxcbiAgICAgIHljb2wgPSBfeSRzaXplMlsxXTtcblxuICB2YXIgbWF0cml4VSA9IFUuX21hdHJpeDtcbiAgdmFyIG1hdHJpeFkgPSB5Ll9tYXRyaXg7XG5cbiAgaWYgKHlyb3cgIT09IHNpemUgfHwgeWNvbCAhPT0gMSkge1xuICAgIHRocm93IG5ldyBFcnJvcihTSVpFX0lOQ09NUEFUSUJMRSk7XG4gIH1cblxuICB2YXIgRVBTSUxPTiA9IDEgLyAoTWF0aC5wb3coMTAsIFUuX2RpZ2l0KSAqIDIpO1xuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgc2l6ZTsgaSsrKSB7XG4gICAgaWYgKE1hdGguYWJzKG1hdHJpeFVbaV1baV0pIDwgRVBTSUxPTikge1xuICAgICAgdGhyb3cgbmV3IEVycm9yKE5PX1VOSVFVRV9TT0xVVElPTik7XG4gICAgfVxuICB9XG5cbiAgdmFyIGNvZWZmaWNpZW50cyA9IGVtcHR5KHNpemUsIDEpO1xuXG4gIGZvciAodmFyIF9pMiA9IHNpemUgLSAxOyBfaTIgPj0gMDsgX2kyLS0pIHtcbiAgICB2YXIgc3VtbWF0aW9uID0gMDtcblxuICAgIGZvciAodmFyIGogPSBfaTIgKyAxOyBqIDwgc2l6ZTsgaisrKSB7XG4gICAgICBzdW1tYXRpb24gKz0gY29lZmZpY2llbnRzW2pdWzBdICogbWF0cml4VVtfaTJdW2pdO1xuICAgIH1cblxuICAgIGNvZWZmaWNpZW50c1tfaTJdWzBdID0gKG1hdHJpeFlbX2kyXVswXSAtIHN1bW1hdGlvbikgLyBtYXRyaXhVW19pMl1bX2kyXTtcbiAgfVxuXG4gIHJldHVybiBuZXcgdGhpcyhjb2VmZmljaWVudHMpO1xufVxuXG47XG5tb2R1bGUuZXhwb3J0cyA9IGJhY2t3YXJkOyIsIlwidXNlIHN0cmljdFwiO1xuXG5mdW5jdGlvbiBfc2xpY2VkVG9BcnJheShhcnIsIGkpIHsgcmV0dXJuIF9hcnJheVdpdGhIb2xlcyhhcnIpIHx8IF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHx8IF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShhcnIsIGkpIHx8IF9ub25JdGVyYWJsZVJlc3QoKTsgfVxuXG5mdW5jdGlvbiBfbm9uSXRlcmFibGVSZXN0KCkgeyB0aHJvdyBuZXcgVHlwZUVycm9yKFwiSW52YWxpZCBhdHRlbXB0IHRvIGRlc3RydWN0dXJlIG5vbi1pdGVyYWJsZSBpbnN0YW5jZS5cXG5JbiBvcmRlciB0byBiZSBpdGVyYWJsZSwgbm9uLWFycmF5IG9iamVjdHMgbXVzdCBoYXZlIGEgW1N5bWJvbC5pdGVyYXRvcl0oKSBtZXRob2QuXCIpOyB9XG5cbmZ1bmN0aW9uIF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShvLCBtaW5MZW4pIHsgaWYgKCFvKSByZXR1cm47IGlmICh0eXBlb2YgbyA9PT0gXCJzdHJpbmdcIikgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IHZhciBuID0gT2JqZWN0LnByb3RvdHlwZS50b1N0cmluZy5jYWxsKG8pLnNsaWNlKDgsIC0xKTsgaWYgKG4gPT09IFwiT2JqZWN0XCIgJiYgby5jb25zdHJ1Y3RvcikgbiA9IG8uY29uc3RydWN0b3IubmFtZTsgaWYgKG4gPT09IFwiTWFwXCIgfHwgbiA9PT0gXCJTZXRcIikgcmV0dXJuIEFycmF5LmZyb20obyk7IGlmIChuID09PSBcIkFyZ3VtZW50c1wiIHx8IC9eKD86VWl8SSludCg/Ojh8MTZ8MzIpKD86Q2xhbXBlZCk/QXJyYXkkLy50ZXN0KG4pKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgfVxuXG5mdW5jdGlvbiBfYXJyYXlMaWtlVG9BcnJheShhcnIsIGxlbikgeyBpZiAobGVuID09IG51bGwgfHwgbGVuID4gYXJyLmxlbmd0aCkgbGVuID0gYXJyLmxlbmd0aDsgZm9yICh2YXIgaSA9IDAsIGFycjIgPSBuZXcgQXJyYXkobGVuKTsgaSA8IGxlbjsgaSsrKSB7IGFycjJbaV0gPSBhcnJbaV07IH0gcmV0dXJuIGFycjI7IH1cblxuZnVuY3Rpb24gX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgeyBpZiAodHlwZW9mIFN5bWJvbCA9PT0gXCJ1bmRlZmluZWRcIiB8fCAhKFN5bWJvbC5pdGVyYXRvciBpbiBPYmplY3QoYXJyKSkpIHJldHVybjsgdmFyIF9hcnIgPSBbXTsgdmFyIF9uID0gdHJ1ZTsgdmFyIF9kID0gZmFsc2U7IHZhciBfZSA9IHVuZGVmaW5lZDsgdHJ5IHsgZm9yICh2YXIgX2kgPSBhcnJbU3ltYm9sLml0ZXJhdG9yXSgpLCBfczsgIShfbiA9IChfcyA9IF9pLm5leHQoKSkuZG9uZSk7IF9uID0gdHJ1ZSkgeyBfYXJyLnB1c2goX3MudmFsdWUpOyBpZiAoaSAmJiBfYXJyLmxlbmd0aCA9PT0gaSkgYnJlYWs7IH0gfSBjYXRjaCAoZXJyKSB7IF9kID0gdHJ1ZTsgX2UgPSBlcnI7IH0gZmluYWxseSB7IHRyeSB7IGlmICghX24gJiYgX2lbXCJyZXR1cm5cIl0gIT0gbnVsbCkgX2lbXCJyZXR1cm5cIl0oKTsgfSBmaW5hbGx5IHsgaWYgKF9kKSB0aHJvdyBfZTsgfSB9IHJldHVybiBfYXJyOyB9XG5cbmZ1bmN0aW9uIF9hcnJheVdpdGhIb2xlcyhhcnIpIHsgaWYgKEFycmF5LmlzQXJyYXkoYXJyKSkgcmV0dXJuIGFycjsgfVxuXG52YXIgZW1wdHkgPSByZXF1aXJlKCcuLi8uLi91dGlsL2VtcHR5Jyk7XG5cbnZhciBfcmVxdWlyZSA9IHJlcXVpcmUoJy4uLy4uL0Vycm9yJyksXG4gICAgSU5WQUxJRF9NQVRSSVggPSBfcmVxdWlyZS5JTlZBTElEX01BVFJJWCxcbiAgICBJTlZBTElEX0xPV0VSX1RSSUFOR1VMQVJfTUFUUklYID0gX3JlcXVpcmUuSU5WQUxJRF9MT1dFUl9UUklBTkdVTEFSX01BVFJJWCxcbiAgICBJTlZBTElEX1NRVUFSRV9NQVRSSVggPSBfcmVxdWlyZS5JTlZBTElEX1NRVUFSRV9NQVRSSVgsXG4gICAgU0laRV9JTkNPTVBBVElCTEUgPSBfcmVxdWlyZS5TSVpFX0lOQ09NUEFUSUJMRSxcbiAgICBOT19VTklRVUVfU09MVVRJT04gPSBfcmVxdWlyZS5OT19VTklRVUVfU09MVVRJT047XG4vKipcclxuICogU29sdmUgc3lzdGVtIG9mIGxpbmVhciBlcXVhdGlvbnMgTHggPSB5IHVzaW5nIGZvcndhcmQgc3Vic3RpdHV0aW9uLFxyXG4gKiB3aGVyZSBMIGlzIGEgbG93ZXIgdHJpYW5ndWxhciBtYXRyaXguXHJcbiAqIElmIHRoZXJlIGlzIG5vIHVuaXF1ZSBzb2x1dGlvbnMsIGFuIGVycm9yIGlzIHRocm93bi5cclxuICogQG1lbWJlcm9mIE1hdHJpeFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7TWF0cml4fSBMIC0gQW55IG4geCBuIGxvd2VyIHRyaWFuZ3VsYXIgTWF0cml4XHJcbiAqIEBwYXJhbSB7TWF0cml4fSB5IC0gQW55IG4geCAxIE1hdHJpeFxyXG4gKiBAcmV0dXJucyB7TWF0cml4fSBuIHggMSBNYXRyaXggd2hpY2ggaXMgdGhlIHNvbHV0aW9uIG9mIEx4ID0geVxyXG4gKi9cblxuXG5mdW5jdGlvbiBmb3J3YXJkKEwsIHkpIHtcbiAgaWYgKCEoTCBpbnN0YW5jZW9mIHRoaXMpIHx8ICEoeSBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfTUFUUklYKTtcbiAgfVxuXG4gIGlmICghTC5pc0xvd2VyVHJpYW5ndWxhcigpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfTE9XRVJfVFJJQU5HVUxBUl9NQVRSSVgpO1xuICB9XG5cbiAgaWYgKCFMLmlzU3F1YXJlKCkpIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoSU5WQUxJRF9TUVVBUkVfTUFUUklYKTtcbiAgfVxuXG4gIHZhciBzaXplID0gTC5zaXplKClbMF07XG5cbiAgdmFyIF95JHNpemUgPSB5LnNpemUoKSxcbiAgICAgIF95JHNpemUyID0gX3NsaWNlZFRvQXJyYXkoX3kkc2l6ZSwgMiksXG4gICAgICB5cm93ID0gX3kkc2l6ZTJbMF0sXG4gICAgICB5Y29sID0gX3kkc2l6ZTJbMV07XG5cbiAgdmFyIG1hdHJpeEwgPSBMLl9tYXRyaXg7XG4gIHZhciBtYXRyaXhZID0geS5fbWF0cml4O1xuXG4gIGlmIChzaXplICE9PSB5cm93IHx8IHljb2wgIT09IDEpIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoU0laRV9JTkNPTVBBVElCTEUpO1xuICB9XG5cbiAgdmFyIEVQU0lMT04gPSAxIC8gKE1hdGgucG93KDEwLCBMLl9kaWdpdCkgKiAyKTtcblxuICBmb3IgKHZhciBpID0gMDsgaSA8IHNpemU7IGkrKykge1xuICAgIGlmIChNYXRoLmFicyhtYXRyaXhMW2ldW2ldKSA8IEVQU0lMT04pIHtcbiAgICAgIHRocm93IG5ldyBFcnJvcihOT19VTklRVUVfU09MVVRJT04pO1xuICAgIH1cbiAgfVxuXG4gIHZhciBjb2VmZmljaWVudHMgPSBlbXB0eShzaXplLCAxKTtcblxuICBmb3IgKHZhciBfaTIgPSAwOyBfaTIgPCBzaXplOyBfaTIrKykge1xuICAgIHZhciBzdW1tYXRpb24gPSAwO1xuXG4gICAgZm9yICh2YXIgaiA9IDA7IGogPCBfaTI7IGorKykge1xuICAgICAgc3VtbWF0aW9uICs9IGNvZWZmaWNpZW50c1tqXVswXSAqIG1hdHJpeExbX2kyXVtqXTtcbiAgICB9XG5cbiAgICBjb2VmZmljaWVudHNbX2kyXVswXSA9IChtYXRyaXhZW19pMl1bMF0gLSBzdW1tYXRpb24pIC8gbWF0cml4TFtfaTJdW19pMl07XG4gIH1cblxuICByZXR1cm4gbmV3IHRoaXMoY29lZmZpY2llbnRzKTtcbn1cblxuO1xubW9kdWxlLmV4cG9ydHMgPSBmb3J3YXJkOyIsIlwidXNlIHN0cmljdFwiO1xuXG5mdW5jdGlvbiBfc2xpY2VkVG9BcnJheShhcnIsIGkpIHsgcmV0dXJuIF9hcnJheVdpdGhIb2xlcyhhcnIpIHx8IF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHx8IF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShhcnIsIGkpIHx8IF9ub25JdGVyYWJsZVJlc3QoKTsgfVxuXG5mdW5jdGlvbiBfbm9uSXRlcmFibGVSZXN0KCkgeyB0aHJvdyBuZXcgVHlwZUVycm9yKFwiSW52YWxpZCBhdHRlbXB0IHRvIGRlc3RydWN0dXJlIG5vbi1pdGVyYWJsZSBpbnN0YW5jZS5cXG5JbiBvcmRlciB0byBiZSBpdGVyYWJsZSwgbm9uLWFycmF5IG9iamVjdHMgbXVzdCBoYXZlIGEgW1N5bWJvbC5pdGVyYXRvcl0oKSBtZXRob2QuXCIpOyB9XG5cbmZ1bmN0aW9uIF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShvLCBtaW5MZW4pIHsgaWYgKCFvKSByZXR1cm47IGlmICh0eXBlb2YgbyA9PT0gXCJzdHJpbmdcIikgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IHZhciBuID0gT2JqZWN0LnByb3RvdHlwZS50b1N0cmluZy5jYWxsKG8pLnNsaWNlKDgsIC0xKTsgaWYgKG4gPT09IFwiT2JqZWN0XCIgJiYgby5jb25zdHJ1Y3RvcikgbiA9IG8uY29uc3RydWN0b3IubmFtZTsgaWYgKG4gPT09IFwiTWFwXCIgfHwgbiA9PT0gXCJTZXRcIikgcmV0dXJuIEFycmF5LmZyb20obyk7IGlmIChuID09PSBcIkFyZ3VtZW50c1wiIHx8IC9eKD86VWl8SSludCg/Ojh8MTZ8MzIpKD86Q2xhbXBlZCk/QXJyYXkkLy50ZXN0KG4pKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgfVxuXG5mdW5jdGlvbiBfYXJyYXlMaWtlVG9BcnJheShhcnIsIGxlbikgeyBpZiAobGVuID09IG51bGwgfHwgbGVuID4gYXJyLmxlbmd0aCkgbGVuID0gYXJyLmxlbmd0aDsgZm9yICh2YXIgaSA9IDAsIGFycjIgPSBuZXcgQXJyYXkobGVuKTsgaSA8IGxlbjsgaSsrKSB7IGFycjJbaV0gPSBhcnJbaV07IH0gcmV0dXJuIGFycjI7IH1cblxuZnVuY3Rpb24gX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgeyBpZiAodHlwZW9mIFN5bWJvbCA9PT0gXCJ1bmRlZmluZWRcIiB8fCAhKFN5bWJvbC5pdGVyYXRvciBpbiBPYmplY3QoYXJyKSkpIHJldHVybjsgdmFyIF9hcnIgPSBbXTsgdmFyIF9uID0gdHJ1ZTsgdmFyIF9kID0gZmFsc2U7IHZhciBfZSA9IHVuZGVmaW5lZDsgdHJ5IHsgZm9yICh2YXIgX2kgPSBhcnJbU3ltYm9sLml0ZXJhdG9yXSgpLCBfczsgIShfbiA9IChfcyA9IF9pLm5leHQoKSkuZG9uZSk7IF9uID0gdHJ1ZSkgeyBfYXJyLnB1c2goX3MudmFsdWUpOyBpZiAoaSAmJiBfYXJyLmxlbmd0aCA9PT0gaSkgYnJlYWs7IH0gfSBjYXRjaCAoZXJyKSB7IF9kID0gdHJ1ZTsgX2UgPSBlcnI7IH0gZmluYWxseSB7IHRyeSB7IGlmICghX24gJiYgX2lbXCJyZXR1cm5cIl0gIT0gbnVsbCkgX2lbXCJyZXR1cm5cIl0oKTsgfSBmaW5hbGx5IHsgaWYgKF9kKSB0aHJvdyBfZTsgfSB9IHJldHVybiBfYXJyOyB9XG5cbmZ1bmN0aW9uIF9hcnJheVdpdGhIb2xlcyhhcnIpIHsgaWYgKEFycmF5LmlzQXJyYXkoYXJyKSkgcmV0dXJuIGFycjsgfVxuXG52YXIgX3JlcXVpcmUgPSByZXF1aXJlKCcuLi8uLi9FcnJvcicpLFxuICAgIElOVkFMSURfTUFUUklYID0gX3JlcXVpcmUuSU5WQUxJRF9NQVRSSVgsXG4gICAgTk9fVU5JUVVFX1NPTFVUSU9OID0gX3JlcXVpcmUuTk9fVU5JUVVFX1NPTFVUSU9OLFxuICAgIElOVkFMSURfU1FVQVJFX01BVFJJWCA9IF9yZXF1aXJlLklOVkFMSURfU1FVQVJFX01BVFJJWCxcbiAgICBTSVpFX0lOQ09NUEFUSUJMRSA9IF9yZXF1aXJlLlNJWkVfSU5DT01QQVRJQkxFO1xuLyoqXHJcbiAqIFNvbHZlIHN5c3RlbSBvZiBsaW5lYXIgZXF1YXRpb25zIEF4ID0geSB1c2luZyBMVSBkZWNvbXBvc2l0aW9uLlxyXG4gKiBJZiB0aGVyZSBpcyBubyB1bmlxdWUgc29sdXRpb25zLCBhbiBlcnJvciBpcyB0aHJvd24uXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge01hdHJpeH0gTCAtIEFueSBuIHggbiBzcXVhcmUgTWF0cml4XHJcbiAqIEBwYXJhbSB7TWF0cml4fSB5IC0gQW55IG4geCAxIE1hdHJpeFxyXG4gKiBAcmV0dXJucyB7TWF0cml4fSBuIHggMSBNYXRyaXggd2hpY2ggaXMgdGhlIHNvbHV0aW9uIG9mIEF4ID0geVxyXG4gKi9cblxuXG5mdW5jdGlvbiBzb2x2ZShBLCBiKSB7XG4gIGlmICghKEEgaW5zdGFuY2VvZiB0aGlzKSB8fCAhKGIgaW5zdGFuY2VvZiB0aGlzKSkge1xuICAgIHRocm93IG5ldyBFcnJvcihJTlZBTElEX01BVFJJWCk7XG4gIH1cblxuICBpZiAoIUEuaXNTcXVhcmUoKSkge1xuICAgIHRocm93IG5ldyBFcnJvcihJTlZBTElEX1NRVUFSRV9NQVRSSVgpO1xuICB9XG5cbiAgdmFyIF9BJHNpemUgPSBBLnNpemUoKSxcbiAgICAgIF9BJHNpemUyID0gX3NsaWNlZFRvQXJyYXkoX0Ekc2l6ZSwgMiksXG4gICAgICBhUm93ID0gX0Ekc2l6ZTJbMF0sXG4gICAgICBhQ29sID0gX0Ekc2l6ZTJbMV07XG5cbiAgdmFyIF9iJHNpemUgPSBiLnNpemUoKSxcbiAgICAgIF9iJHNpemUyID0gX3NsaWNlZFRvQXJyYXkoX2Ikc2l6ZSwgMiksXG4gICAgICBiUm93ID0gX2Ikc2l6ZTJbMF0sXG4gICAgICBiQ29sID0gX2Ikc2l6ZTJbMV07XG5cbiAgaWYgKGFDb2wgIT09IGJSb3cgfHwgYkNvbCAhPT0gMSkge1xuICAgIHRocm93IG5ldyBFcnJvcihTSVpFX0lOQ09NUEFUSUJMRSk7XG4gIH1cblxuICB2YXIgRVBTSUxPTiA9IDEgLyAoTWF0aC5wb3coMTAsIEEuX2RpZ2l0KSAqIDIpO1xuXG4gIHZhciBfdGhpcyRMVSA9IHRoaXMuTFUoQSwgdHJ1ZSksXG4gICAgICBfdGhpcyRMVTIgPSBfc2xpY2VkVG9BcnJheShfdGhpcyRMVSwgMiksXG4gICAgICBQID0gX3RoaXMkTFUyWzBdLFxuICAgICAgTFUgPSBfdGhpcyRMVTJbMV07XG5cbiAgdmFyIG1hdHJpeExVID0gTFUuX21hdHJpeDtcbiAgdmFyIG1hdHJpeEIgPSBiLl9tYXRyaXg7XG5cbiAgZm9yICh2YXIgaSA9IGFSb3cgLSAxOyBpID49IDA7IGktLSkge1xuICAgIGlmIChNYXRoLmFicyhtYXRyaXhMVVtpXVtpXSkgPCBFUFNJTE9OKSB7XG4gICAgICB0aHJvdyBuZXcgRXJyb3IoTk9fVU5JUVVFX1NPTFVUSU9OKTtcbiAgICB9XG4gIH1cblxuICB2YXIgY2xvbmVkVmVjdG9yID0gbmV3IEFycmF5KGJSb3cpO1xuICB2YXIgY29lZmZpY2llbnRzID0gbmV3IEFycmF5KGJSb3cpO1xuXG4gIGZvciAodmFyIF9pMiA9IDA7IF9pMiA8IGJSb3c7IF9pMisrKSB7XG4gICAgLy8gZXNsaW50LWRpc2FibGUtbmV4dC1saW5lIHByZWZlci1kZXN0cnVjdHVyaW5nXG4gICAgY2xvbmVkVmVjdG9yW19pMl0gPSBtYXRyaXhCW1BbX2kyXV1bMF07XG4gIH1cblxuICBmb3IgKHZhciBfaTMgPSAwOyBfaTMgPCBhUm93OyBfaTMrKykge1xuICAgIHZhciBzdW1tYXRpb24gPSAwO1xuXG4gICAgZm9yICh2YXIgaiA9IDA7IGogPCBfaTM7IGorKykge1xuICAgICAgc3VtbWF0aW9uICs9IGNvZWZmaWNpZW50c1tqXSAqIG1hdHJpeExVW19pM11bal07XG4gICAgfVxuXG4gICAgY29lZmZpY2llbnRzW19pM10gPSBjbG9uZWRWZWN0b3JbX2kzXSAtIHN1bW1hdGlvbjtcbiAgfVxuXG4gIGZvciAodmFyIF9pNCA9IGFSb3cgLSAxOyBfaTQgPj0gMDsgX2k0LS0pIHtcbiAgICB2YXIgX3N1bW1hdGlvbiA9IDA7XG5cbiAgICBmb3IgKHZhciBfaiA9IF9pNCArIDE7IF9qIDwgYVJvdzsgX2orKykge1xuICAgICAgX3N1bW1hdGlvbiArPSBtYXRyaXhMVVtfaTRdW19qXSAqIGNsb25lZFZlY3Rvcltfal07XG4gICAgfVxuXG4gICAgY2xvbmVkVmVjdG9yW19pNF0gPSAoY29lZmZpY2llbnRzW19pNF0gLSBfc3VtbWF0aW9uKSAvIG1hdHJpeExVW19pNF1bX2k0XTtcbiAgfVxuXG4gIGZvciAodmFyIF9pNSA9IDA7IF9pNSA8IGJSb3c7IF9pNSsrKSB7XG4gICAgY29lZmZpY2llbnRzW19pNV0gPSBbY2xvbmVkVmVjdG9yW19pNV1dO1xuICB9XG5cbiAgcmV0dXJuIG5ldyB0aGlzKGNvZWZmaWNpZW50cyk7XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gc29sdmU7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbmZ1bmN0aW9uIF9zbGljZWRUb0FycmF5KGFyciwgaSkgeyByZXR1cm4gX2FycmF5V2l0aEhvbGVzKGFycikgfHwgX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgfHwgX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KGFyciwgaSkgfHwgX25vbkl0ZXJhYmxlUmVzdCgpOyB9XG5cbmZ1bmN0aW9uIF9ub25JdGVyYWJsZVJlc3QoKSB7IHRocm93IG5ldyBUeXBlRXJyb3IoXCJJbnZhbGlkIGF0dGVtcHQgdG8gZGVzdHJ1Y3R1cmUgbm9uLWl0ZXJhYmxlIGluc3RhbmNlLlxcbkluIG9yZGVyIHRvIGJlIGl0ZXJhYmxlLCBub24tYXJyYXkgb2JqZWN0cyBtdXN0IGhhdmUgYSBbU3ltYm9sLml0ZXJhdG9yXSgpIG1ldGhvZC5cIik7IH1cblxuZnVuY3Rpb24gX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KG8sIG1pbkxlbikgeyBpZiAoIW8pIHJldHVybjsgaWYgKHR5cGVvZiBvID09PSBcInN0cmluZ1wiKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgdmFyIG4gPSBPYmplY3QucHJvdG90eXBlLnRvU3RyaW5nLmNhbGwobykuc2xpY2UoOCwgLTEpOyBpZiAobiA9PT0gXCJPYmplY3RcIiAmJiBvLmNvbnN0cnVjdG9yKSBuID0gby5jb25zdHJ1Y3Rvci5uYW1lOyBpZiAobiA9PT0gXCJNYXBcIiB8fCBuID09PSBcIlNldFwiKSByZXR1cm4gQXJyYXkuZnJvbShvKTsgaWYgKG4gPT09IFwiQXJndW1lbnRzXCIgfHwgL14oPzpVaXxJKW50KD86OHwxNnwzMikoPzpDbGFtcGVkKT9BcnJheSQvLnRlc3QobikpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB9XG5cbmZ1bmN0aW9uIF9hcnJheUxpa2VUb0FycmF5KGFyciwgbGVuKSB7IGlmIChsZW4gPT0gbnVsbCB8fCBsZW4gPiBhcnIubGVuZ3RoKSBsZW4gPSBhcnIubGVuZ3RoOyBmb3IgKHZhciBpID0gMCwgYXJyMiA9IG5ldyBBcnJheShsZW4pOyBpIDwgbGVuOyBpKyspIHsgYXJyMltpXSA9IGFycltpXTsgfSByZXR1cm4gYXJyMjsgfVxuXG5mdW5jdGlvbiBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB7IGlmICh0eXBlb2YgU3ltYm9sID09PSBcInVuZGVmaW5lZFwiIHx8ICEoU3ltYm9sLml0ZXJhdG9yIGluIE9iamVjdChhcnIpKSkgcmV0dXJuOyB2YXIgX2FyciA9IFtdOyB2YXIgX24gPSB0cnVlOyB2YXIgX2QgPSBmYWxzZTsgdmFyIF9lID0gdW5kZWZpbmVkOyB0cnkgeyBmb3IgKHZhciBfaSA9IGFycltTeW1ib2wuaXRlcmF0b3JdKCksIF9zOyAhKF9uID0gKF9zID0gX2kubmV4dCgpKS5kb25lKTsgX24gPSB0cnVlKSB7IF9hcnIucHVzaChfcy52YWx1ZSk7IGlmIChpICYmIF9hcnIubGVuZ3RoID09PSBpKSBicmVhazsgfSB9IGNhdGNoIChlcnIpIHsgX2QgPSB0cnVlOyBfZSA9IGVycjsgfSBmaW5hbGx5IHsgdHJ5IHsgaWYgKCFfbiAmJiBfaVtcInJldHVyblwiXSAhPSBudWxsKSBfaVtcInJldHVyblwiXSgpOyB9IGZpbmFsbHkgeyBpZiAoX2QpIHRocm93IF9lOyB9IH0gcmV0dXJuIF9hcnI7IH1cblxuZnVuY3Rpb24gX2FycmF5V2l0aEhvbGVzKGFycikgeyBpZiAoQXJyYXkuaXNBcnJheShhcnIpKSByZXR1cm4gYXJyOyB9XG5cbnZhciBfcmVxdWlyZSA9IHJlcXVpcmUoJy4uLy4uL0Vycm9yJyksXG4gICAgSU5WQUxJRF9NQVRSSVggPSBfcmVxdWlyZS5JTlZBTElEX01BVFJJWCxcbiAgICBTSVpFX0lOQ09NUEFUSUJMRSA9IF9yZXF1aXJlLlNJWkVfSU5DT01QQVRJQkxFO1xuLyoqXHJcbiAqIENhbGN1bGF0ZXMgdGhlIHN1bSBvZiB0d28gTWF0cmljZXMuXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge01hdHJpeH0gQSAtIEFueSBNYXRyaXhcclxuICogQHBhcmFtIHtNYXRyaXh9IEIgLSBBbnkgTWF0cml4IHRoYXQgaGFzIHNhbWUgc2l6ZSB3aXRoIEFcclxuICogQHJldHVybnMge01hdHJpeH0gVGhlIHN1bSBvZiB0d28gTWF0cmljZXNcclxuICovXG5cblxuZnVuY3Rpb24gYWRkKEEsIEIpIHtcbiAgaWYgKCEoQSBpbnN0YW5jZW9mIHRoaXMpIHx8ICEoQiBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfTUFUUklYKTtcbiAgfVxuXG4gIHZhciBfQSRzaXplID0gQS5zaXplKCksXG4gICAgICBfQSRzaXplMiA9IF9zbGljZWRUb0FycmF5KF9BJHNpemUsIDIpLFxuICAgICAgcm93ID0gX0Ekc2l6ZTJbMF0sXG4gICAgICBjb2wgPSBfQSRzaXplMlsxXTtcblxuICB2YXIgX0Ikc2l6ZSA9IEIuc2l6ZSgpLFxuICAgICAgX0Ikc2l6ZTIgPSBfc2xpY2VkVG9BcnJheShfQiRzaXplLCAyKSxcbiAgICAgIHJvdzIgPSBfQiRzaXplMlswXSxcbiAgICAgIGNvbDIgPSBfQiRzaXplMlsxXTtcblxuICBpZiAocm93ICE9PSByb3cyIHx8IGNvbCAhPT0gY29sMikge1xuICAgIHRocm93IG5ldyBFcnJvcihTSVpFX0lOQ09NUEFUSUJMRSk7XG4gIH1cblxuICB2YXIgbWF0cml4MSA9IEEuX21hdHJpeDtcbiAgdmFyIG1hdHJpeDIgPSBCLl9tYXRyaXg7XG4gIHJldHVybiB0aGlzLmdlbmVyYXRlKHJvdywgY29sLCBmdW5jdGlvbiAoaSwgaikge1xuICAgIHJldHVybiBtYXRyaXgxW2ldW2pdICsgbWF0cml4MltpXVtqXTtcbiAgfSk7XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gYWRkOyIsIlwidXNlIHN0cmljdFwiO1xuXG52YXIgX3JlcXVpcmUgPSByZXF1aXJlKCcuLi8uLi9FcnJvcicpLFxuICAgIElOVkFMSURfTUFUUklYID0gX3JlcXVpcmUuSU5WQUxJRF9NQVRSSVgsXG4gICAgSU5WQUxJRF9TUVVBUkVfTUFUUklYID0gX3JlcXVpcmUuSU5WQUxJRF9TUVVBUkVfTUFUUklYLFxuICAgIFNJTkdVTEFSX01BVFJJWCA9IF9yZXF1aXJlLlNJTkdVTEFSX01BVFJJWDtcblxudmFyIE1hdHJpeCA9IHJlcXVpcmUoJy4uLy4uJyk7XG4vKipcclxuICogRmluZCB0aGUgaW52ZXJzZSBvZiBub24tc2luZ3VsYXIgbWF0cml4IHVzaW5nIEVsZW1lbnRhcnkgUm93IE9wZXJhdGlvbnMuXHJcbiAqIElmIHRoZSBtYXRyaXggaXMgc2luZ3VsYXIsIGFuIGVycm9yIGlzIHRocm93bi5cclxuICogQG1lbWJlcm9mIE1hdHJpeFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7TWF0cml4fSBBIC0gQW55IHNxdWFyZSBNYXRyaXhcclxuICogQHJldHVybnMge01hdHJpeH0gVGhlIGludmVyc2Ugb2YgQVxyXG4gKi9cblxuXG5mdW5jdGlvbiBpbnZlcnNlKEEpIHtcbiAgaWYgKCEoQSBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfTUFUUklYKTtcbiAgfVxuXG4gIGlmICghQS5pc1NxdWFyZSgpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfU1FVQVJFX01BVFJJWCk7XG4gIH1cblxuICB2YXIgc2l6ZSA9IEEuc2l6ZSgpWzBdO1xuXG4gIGlmIChzaXplID09PSAwKSB7XG4gICAgLy8gaW52ZXJzZSBvZiAweDAgbWF0cml4IGlzIGl0c2VsZlxuICAgIHJldHVybiBuZXcgTWF0cml4KFtdKTtcbiAgfVxuXG4gIHZhciBFUFNJTE9OID0gMSAvIChNYXRoLnBvdygxMCwgQS5fZGlnaXQpICogMik7XG5cbiAgdmFyIGludiA9IHRoaXMuaWRlbnRpdHkoc2l6ZSkuX21hdHJpeDtcblxuICB2YXIgY2xvbmUgPSB0aGlzLmNsb25lKEEpLl9tYXRyaXg7XG5cbiAgdmFyIHBlcm11dGF0aW9uID0gaW5pdFBlcm11dGF0aW9uKHNpemUpOyAvLyBpdGVyYXRlIGVhY2ggY29sdW1uXG5cbiAgZm9yICh2YXIgaiA9IDA7IGogPCBzaXplOyBqKyspIHtcbiAgICB2YXIgcGl2b3RJZHggPSBqO1xuICAgIHZhciBwaXZvdCA9IGNsb25lW3Blcm11dGF0aW9uW2pdXVtqXTtcblxuICAgIHdoaWxlIChNYXRoLmFicyhwaXZvdCkgPCBFUFNJTE9OICYmIHBpdm90SWR4IDwgc2l6ZSAtIDEpIHtcbiAgICAgIHBpdm90SWR4Kys7XG4gICAgICBwaXZvdCA9IGNsb25lW3Blcm11dGF0aW9uW3Bpdm90SWR4XV1bal07XG4gICAgfVxuXG4gICAgaWYgKE1hdGguYWJzKHBpdm90KSA8IEVQU0lMT04pIHtcbiAgICAgIHRocm93IG5ldyBFcnJvcihTSU5HVUxBUl9NQVRSSVgpO1xuICAgIH1cblxuICAgIGlmIChqICE9PSBwaXZvdElkeCkge1xuICAgICAgdmFyIHRlbXAgPSBwZXJtdXRhdGlvbltqXTtcbiAgICAgIHBlcm11dGF0aW9uW2pdID0gcGVybXV0YXRpb25bcGl2b3RJZHhdO1xuICAgICAgcGVybXV0YXRpb25bcGl2b3RJZHhdID0gdGVtcDtcbiAgICB9XG5cbiAgICB2YXIgcGl2b3RSb3cgPSBwZXJtdXRhdGlvbltqXTsgLy8gdGhlIHBpdm90IGlzIGd1YXJhbnRlZWQgdG8gYmUgbm9uLXplcm9cblxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgc2l6ZTsgaSsrKSB7XG4gICAgICB2YXIgaXRoID0gcGVybXV0YXRpb25baV07XG5cbiAgICAgIGlmIChpID09PSBqKSB7XG4gICAgICAgIGZvciAodmFyIGsgPSAwOyBrIDwgc2l6ZTsgaysrKSB7XG4gICAgICAgICAgaWYgKGsgPT09IGopIHtcbiAgICAgICAgICAgIGNsb25lW2l0aF1ba10gPSAxO1xuICAgICAgICAgIH1cblxuICAgICAgICAgIGlmIChrID4gaikge1xuICAgICAgICAgICAgY2xvbmVbaXRoXVtrXSAvPSBwaXZvdDtcbiAgICAgICAgICB9XG5cbiAgICAgICAgICBpbnZbaXRoXVtrXSAvPSBwaXZvdDtcbiAgICAgICAgfVxuXG4gICAgICAgIHBpdm90ID0gMTtcbiAgICAgIH1cblxuICAgICAgaWYgKGkgIT09IGogJiYgTWF0aC5hYnMoY2xvbmVbaXRoXVtqXSkgPj0gRVBTSUxPTikge1xuICAgICAgICB2YXIgZmFjdG9yID0gY2xvbmVbaXRoXVtqXSAvIHBpdm90O1xuXG4gICAgICAgIGZvciAodmFyIF9rID0gMDsgX2sgPCBzaXplOyBfaysrKSB7XG4gICAgICAgICAgaWYgKF9rID09PSBqKSB7XG4gICAgICAgICAgICBjbG9uZVtpdGhdW19rXSA9IDA7XG4gICAgICAgICAgfVxuXG4gICAgICAgICAgaWYgKF9rID4gaikge1xuICAgICAgICAgICAgY2xvbmVbaXRoXVtfa10gLT0gZmFjdG9yICogY2xvbmVbcGl2b3RSb3ddW19rXTtcbiAgICAgICAgICB9XG5cbiAgICAgICAgICBpbnZbaXRoXVtfa10gLT0gZmFjdG9yICogaW52W3Bpdm90Um93XVtfa107XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICB9XG4gIH1cblxuICBmb3IgKHZhciBfaSA9IDA7IF9pIDwgc2l6ZTsgX2krKykge1xuICAgIGNsb25lW19pXSA9IGludltwZXJtdXRhdGlvbltfaV1dO1xuICB9XG5cbiAgcmV0dXJuIG5ldyB0aGlzKGNsb25lKTtcbn1cblxuO1xuXG5mdW5jdGlvbiBpbml0UGVybXV0YXRpb24oc2l6ZSkge1xuICB2YXIgcGVybXV0YXRpb24gPSBuZXcgQXJyYXkoc2l6ZSk7XG5cbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBzaXplOyBpKyspIHtcbiAgICBwZXJtdXRhdGlvbltpXSA9IGk7XG4gIH1cblxuICByZXR1cm4gcGVybXV0YXRpb247XG59XG5cbm1vZHVsZS5leHBvcnRzID0gaW52ZXJzZTsiLCJcInVzZSBzdHJpY3RcIjtcblxuZnVuY3Rpb24gX3NsaWNlZFRvQXJyYXkoYXJyLCBpKSB7IHJldHVybiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB8fCBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB8fCBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkoYXJyLCBpKSB8fCBfbm9uSXRlcmFibGVSZXN0KCk7IH1cblxuZnVuY3Rpb24gX25vbkl0ZXJhYmxlUmVzdCgpIHsgdGhyb3cgbmV3IFR5cGVFcnJvcihcIkludmFsaWQgYXR0ZW1wdCB0byBkZXN0cnVjdHVyZSBub24taXRlcmFibGUgaW5zdGFuY2UuXFxuSW4gb3JkZXIgdG8gYmUgaXRlcmFibGUsIG5vbi1hcnJheSBvYmplY3RzIG11c3QgaGF2ZSBhIFtTeW1ib2wuaXRlcmF0b3JdKCkgbWV0aG9kLlwiKTsgfVxuXG5mdW5jdGlvbiBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkobywgbWluTGVuKSB7IGlmICghbykgcmV0dXJuOyBpZiAodHlwZW9mIG8gPT09IFwic3RyaW5nXCIpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB2YXIgbiA9IE9iamVjdC5wcm90b3R5cGUudG9TdHJpbmcuY2FsbChvKS5zbGljZSg4LCAtMSk7IGlmIChuID09PSBcIk9iamVjdFwiICYmIG8uY29uc3RydWN0b3IpIG4gPSBvLmNvbnN0cnVjdG9yLm5hbWU7IGlmIChuID09PSBcIk1hcFwiIHx8IG4gPT09IFwiU2V0XCIpIHJldHVybiBBcnJheS5mcm9tKG8pOyBpZiAobiA9PT0gXCJBcmd1bWVudHNcIiB8fCAvXig/OlVpfEkpbnQoPzo4fDE2fDMyKSg/OkNsYW1wZWQpP0FycmF5JC8udGVzdChuKSkgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IH1cblxuZnVuY3Rpb24gX2FycmF5TGlrZVRvQXJyYXkoYXJyLCBsZW4pIHsgaWYgKGxlbiA9PSBudWxsIHx8IGxlbiA+IGFyci5sZW5ndGgpIGxlbiA9IGFyci5sZW5ndGg7IGZvciAodmFyIGkgPSAwLCBhcnIyID0gbmV3IEFycmF5KGxlbik7IGkgPCBsZW47IGkrKykgeyBhcnIyW2ldID0gYXJyW2ldOyB9IHJldHVybiBhcnIyOyB9XG5cbmZ1bmN0aW9uIF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHsgaWYgKHR5cGVvZiBTeW1ib2wgPT09IFwidW5kZWZpbmVkXCIgfHwgIShTeW1ib2wuaXRlcmF0b3IgaW4gT2JqZWN0KGFycikpKSByZXR1cm47IHZhciBfYXJyID0gW107IHZhciBfbiA9IHRydWU7IHZhciBfZCA9IGZhbHNlOyB2YXIgX2UgPSB1bmRlZmluZWQ7IHRyeSB7IGZvciAodmFyIF9pID0gYXJyW1N5bWJvbC5pdGVyYXRvcl0oKSwgX3M7ICEoX24gPSAoX3MgPSBfaS5uZXh0KCkpLmRvbmUpOyBfbiA9IHRydWUpIHsgX2Fyci5wdXNoKF9zLnZhbHVlKTsgaWYgKGkgJiYgX2Fyci5sZW5ndGggPT09IGkpIGJyZWFrOyB9IH0gY2F0Y2ggKGVycikgeyBfZCA9IHRydWU7IF9lID0gZXJyOyB9IGZpbmFsbHkgeyB0cnkgeyBpZiAoIV9uICYmIF9pW1wicmV0dXJuXCJdICE9IG51bGwpIF9pW1wicmV0dXJuXCJdKCk7IH0gZmluYWxseSB7IGlmIChfZCkgdGhyb3cgX2U7IH0gfSByZXR1cm4gX2FycjsgfVxuXG5mdW5jdGlvbiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB7IGlmIChBcnJheS5pc0FycmF5KGFycikpIHJldHVybiBhcnI7IH1cblxudmFyIGVtcHR5ID0gcmVxdWlyZSgnLi4vLi4vdXRpbC9lbXB0eScpO1xuXG52YXIgX3JlcXVpcmUgPSByZXF1aXJlKCcuLi8uLi9FcnJvcicpLFxuICAgIElOVkFMSURfTUFUUklYID0gX3JlcXVpcmUuSU5WQUxJRF9NQVRSSVgsXG4gICAgU0laRV9JTkNPTVBBVElCTEUgPSBfcmVxdWlyZS5TSVpFX0lOQ09NUEFUSUJMRTtcbi8qKlxyXG4gKiBDYWxjdWxhdGVzIHRoZSBwcm9kdWN0IG9mIHR3byBNYXRyaWNlcy5cclxuICogQG1lbWJlcm9mIE1hdHJpeFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7TWF0cml4fSBBIC0gQW55IE1hdHJpeFxyXG4gKiBAcGFyYW0ge01hdHJpeH0gQiAtIEFueSBNYXRyaXggdGhhdCBpcyBzaXplLWNvbXBhdGlibGUgd2l0aCBBXHJcbiAqIEByZXR1cm5zIHtNYXRyaXh9IFRoZSBwcm9kdWN0IG9mIHR3byBNYXRyaWNlc1xyXG4gKi9cblxuXG5mdW5jdGlvbiBtdWx0aXBseShBLCBCKSB7XG4gIGlmICghKEEgaW5zdGFuY2VvZiB0aGlzKSB8fCAhKEIgaW5zdGFuY2VvZiB0aGlzKSkge1xuICAgIHRocm93IG5ldyBFcnJvcihJTlZBTElEX01BVFJJWCk7XG4gIH1cblxuICB2YXIgX0Ekc2l6ZSA9IEEuc2l6ZSgpLFxuICAgICAgX0Ekc2l6ZTIgPSBfc2xpY2VkVG9BcnJheShfQSRzaXplLCAyKSxcbiAgICAgIEFyb3cgPSBfQSRzaXplMlswXSxcbiAgICAgIEFjb2wgPSBfQSRzaXplMlsxXTtcblxuICB2YXIgX0Ikc2l6ZSA9IEIuc2l6ZSgpLFxuICAgICAgX0Ikc2l6ZTIgPSBfc2xpY2VkVG9BcnJheShfQiRzaXplLCAyKSxcbiAgICAgIEJyb3cgPSBfQiRzaXplMlswXSxcbiAgICAgIEJjb2wgPSBfQiRzaXplMlsxXTtcblxuICBpZiAoQWNvbCAhPT0gQnJvdykge1xuICAgIHRocm93IG5ldyBFcnJvcihTSVpFX0lOQ09NUEFUSUJMRSk7XG4gIH1cblxuICB2YXIgbWF0cml4QSA9IEEuX21hdHJpeDtcbiAgdmFyIG1hdHJpeEIgPSBCLl9tYXRyaXg7XG4gIHZhciByZXN1bHQgPSBlbXB0eShBcm93LCBCY29sKTtcblxuICBmb3IgKHZhciBpID0gMDsgaSA8IEFyb3c7IGkrKykge1xuICAgIGZvciAodmFyIGogPSAwOyBqIDwgQmNvbDsgaisrKSB7XG4gICAgICByZXN1bHRbaV1bal0gPSAwO1xuXG4gICAgICBmb3IgKHZhciBrID0gMDsgayA8IEJyb3c7IGsrKykge1xuICAgICAgICByZXN1bHRbaV1bal0gKz0gbWF0cml4QVtpXVtrXSAqIG1hdHJpeEJba11bal07XG4gICAgICB9XG4gICAgfVxuICB9XG5cbiAgcmV0dXJuIG5ldyB0aGlzKHJlc3VsdCk7XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gbXVsdGlwbHk7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbnZhciBfcmVxdWlyZSA9IHJlcXVpcmUoJy4uLy4uL0Vycm9yJyksXG4gICAgSU5WQUxJRF9NQVRSSVggPSBfcmVxdWlyZS5JTlZBTElEX01BVFJJWCxcbiAgICBJTlZBTElEX1NRVUFSRV9NQVRSSVggPSBfcmVxdWlyZS5JTlZBTElEX1NRVUFSRV9NQVRSSVgsXG4gICAgSU5WQUxJRF9FWFBPTkVOVCA9IF9yZXF1aXJlLklOVkFMSURfRVhQT05FTlQ7XG4vKipcclxuICogQ2FsY3VsYXRlcyB0aGUgcG93ZXIgb2YgYW55IHNxdWFyZSBtYXRyaXguXHJcbiAqIFRoZSBhbGdvcml0aG0gaXMgaW1wbGVtZW50ZWQgcmVjdXJzaXZlbHkuXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge01hdHJpeH0gQSAtIEFueSBzcXVhcmUgTWF0cml4XHJcbiAqIEBwYXJhbSB7bnVtYmVyfSBleHBvbmVudCAtIEFueSBOb24tbmVnYXRpdmUgaW50ZWdlclxyXG4gKiBAcmV0dXJucyB7TWF0cml4fSBUaGUgcG93ZXIgb2YgQVxyXG4gKi9cblxuXG5mdW5jdGlvbiBwb3coQSwgZXhwb25lbnQpIHtcbiAgaWYgKCEoQSBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfTUFUUklYKTtcbiAgfVxuXG4gIGlmICghQS5pc1NxdWFyZSgpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfU1FVQVJFX01BVFJJWCk7XG4gIH1cblxuICBpZiAoIU51bWJlci5pc0ludGVnZXIoZXhwb25lbnQpIHx8IGV4cG9uZW50IDwgMCkge1xuICAgIHRocm93IG5ldyBFcnJvcihJTlZBTElEX0VYUE9ORU5UKTtcbiAgfVxuXG4gIHZhciBzaXplID0gQS5zaXplKClbMF07XG5cbiAgaWYgKGV4cG9uZW50ID09PSAwKSB7XG4gICAgcmV0dXJuIHRoaXMuaWRlbnRpdHkoc2l6ZSk7XG4gIH1cblxuICBpZiAoZXhwb25lbnQgPT09IDEpIHtcbiAgICByZXR1cm4gdGhpcy5jbG9uZShBKTtcbiAgfVxuXG4gIGlmIChleHBvbmVudCAlIDIgPT09IDApIHtcbiAgICB2YXIgX3RlbXAgPSB0aGlzLnBvdyhBLCBleHBvbmVudCAvIDIpO1xuXG4gICAgcmV0dXJuIHRoaXMubXVsdGlwbHkoX3RlbXAsIF90ZW1wKTtcbiAgfVxuXG4gIHZhciB0ZW1wID0gdGhpcy5wb3coQSwgKGV4cG9uZW50IC0gMSkgLyAyKTtcbiAgcmV0dXJuIHRoaXMubXVsdGlwbHkodGhpcy5tdWx0aXBseSh0ZW1wLCB0ZW1wKSwgQSk7XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gcG93OyIsIlwidXNlIHN0cmljdFwiO1xuXG5mdW5jdGlvbiBfc2xpY2VkVG9BcnJheShhcnIsIGkpIHsgcmV0dXJuIF9hcnJheVdpdGhIb2xlcyhhcnIpIHx8IF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHx8IF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShhcnIsIGkpIHx8IF9ub25JdGVyYWJsZVJlc3QoKTsgfVxuXG5mdW5jdGlvbiBfbm9uSXRlcmFibGVSZXN0KCkgeyB0aHJvdyBuZXcgVHlwZUVycm9yKFwiSW52YWxpZCBhdHRlbXB0IHRvIGRlc3RydWN0dXJlIG5vbi1pdGVyYWJsZSBpbnN0YW5jZS5cXG5JbiBvcmRlciB0byBiZSBpdGVyYWJsZSwgbm9uLWFycmF5IG9iamVjdHMgbXVzdCBoYXZlIGEgW1N5bWJvbC5pdGVyYXRvcl0oKSBtZXRob2QuXCIpOyB9XG5cbmZ1bmN0aW9uIF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShvLCBtaW5MZW4pIHsgaWYgKCFvKSByZXR1cm47IGlmICh0eXBlb2YgbyA9PT0gXCJzdHJpbmdcIikgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IHZhciBuID0gT2JqZWN0LnByb3RvdHlwZS50b1N0cmluZy5jYWxsKG8pLnNsaWNlKDgsIC0xKTsgaWYgKG4gPT09IFwiT2JqZWN0XCIgJiYgby5jb25zdHJ1Y3RvcikgbiA9IG8uY29uc3RydWN0b3IubmFtZTsgaWYgKG4gPT09IFwiTWFwXCIgfHwgbiA9PT0gXCJTZXRcIikgcmV0dXJuIEFycmF5LmZyb20obyk7IGlmIChuID09PSBcIkFyZ3VtZW50c1wiIHx8IC9eKD86VWl8SSludCg/Ojh8MTZ8MzIpKD86Q2xhbXBlZCk/QXJyYXkkLy50ZXN0KG4pKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgfVxuXG5mdW5jdGlvbiBfYXJyYXlMaWtlVG9BcnJheShhcnIsIGxlbikgeyBpZiAobGVuID09IG51bGwgfHwgbGVuID4gYXJyLmxlbmd0aCkgbGVuID0gYXJyLmxlbmd0aDsgZm9yICh2YXIgaSA9IDAsIGFycjIgPSBuZXcgQXJyYXkobGVuKTsgaSA8IGxlbjsgaSsrKSB7IGFycjJbaV0gPSBhcnJbaV07IH0gcmV0dXJuIGFycjI7IH1cblxuZnVuY3Rpb24gX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgeyBpZiAodHlwZW9mIFN5bWJvbCA9PT0gXCJ1bmRlZmluZWRcIiB8fCAhKFN5bWJvbC5pdGVyYXRvciBpbiBPYmplY3QoYXJyKSkpIHJldHVybjsgdmFyIF9hcnIgPSBbXTsgdmFyIF9uID0gdHJ1ZTsgdmFyIF9kID0gZmFsc2U7IHZhciBfZSA9IHVuZGVmaW5lZDsgdHJ5IHsgZm9yICh2YXIgX2kgPSBhcnJbU3ltYm9sLml0ZXJhdG9yXSgpLCBfczsgIShfbiA9IChfcyA9IF9pLm5leHQoKSkuZG9uZSk7IF9uID0gdHJ1ZSkgeyBfYXJyLnB1c2goX3MudmFsdWUpOyBpZiAoaSAmJiBfYXJyLmxlbmd0aCA9PT0gaSkgYnJlYWs7IH0gfSBjYXRjaCAoZXJyKSB7IF9kID0gdHJ1ZTsgX2UgPSBlcnI7IH0gZmluYWxseSB7IHRyeSB7IGlmICghX24gJiYgX2lbXCJyZXR1cm5cIl0gIT0gbnVsbCkgX2lbXCJyZXR1cm5cIl0oKTsgfSBmaW5hbGx5IHsgaWYgKF9kKSB0aHJvdyBfZTsgfSB9IHJldHVybiBfYXJyOyB9XG5cbmZ1bmN0aW9uIF9hcnJheVdpdGhIb2xlcyhhcnIpIHsgaWYgKEFycmF5LmlzQXJyYXkoYXJyKSkgcmV0dXJuIGFycjsgfVxuXG52YXIgX3JlcXVpcmUgPSByZXF1aXJlKCcuLi8uLi9FcnJvcicpLFxuICAgIFNJWkVfSU5DT01QQVRJQkxFID0gX3JlcXVpcmUuU0laRV9JTkNPTVBBVElCTEUsXG4gICAgSU5WQUxJRF9NQVRSSVggPSBfcmVxdWlyZS5JTlZBTElEX01BVFJJWDtcbi8qKlxyXG4gKiBDYWxjdWxhdGVzIHRoZSBkaWZmZXJlbmNlIG9mIHR3byBNYXRyaWNlcy5cclxuICogQG1lbWJlcm9mIE1hdHJpeFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7TWF0cml4fSBBIC0gQW55IE1hdHJpeFxyXG4gKiBAcGFyYW0ge01hdHJpeH0gQiAtIEFueSBNYXRyaXggdGhhdCBoYXMgc2FtZSBzaXplIHdpdGggQVxyXG4gKiBAcmV0dXJucyB7TWF0cml4fSBUaGUgZGlmZmVyZW5jZSBvZiB0d28gTWF0cmljZXNcclxuICovXG5cblxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiBzdWJ0cmFjdChBLCBCKSB7XG4gIGlmICghKEEgaW5zdGFuY2VvZiB0aGlzKSB8fCAhKEIgaW5zdGFuY2VvZiB0aGlzKSkge1xuICAgIHRocm93IG5ldyBFcnJvcihJTlZBTElEX01BVFJJWCk7XG4gIH1cblxuICB2YXIgX0Ekc2l6ZSA9IEEuc2l6ZSgpLFxuICAgICAgX0Ekc2l6ZTIgPSBfc2xpY2VkVG9BcnJheShfQSRzaXplLCAyKSxcbiAgICAgIHJvdyA9IF9BJHNpemUyWzBdLFxuICAgICAgY29sID0gX0Ekc2l6ZTJbMV07XG5cbiAgdmFyIF9CJHNpemUgPSBCLnNpemUoKSxcbiAgICAgIF9CJHNpemUyID0gX3NsaWNlZFRvQXJyYXkoX0Ikc2l6ZSwgMiksXG4gICAgICByb3cyID0gX0Ikc2l6ZTJbMF0sXG4gICAgICBjb2wyID0gX0Ikc2l6ZTJbMV07XG5cbiAgaWYgKHJvdyAhPT0gcm93MiB8fCBjb2wgIT09IGNvbDIpIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoU0laRV9JTkNPTVBBVElCTEUpO1xuICB9XG5cbiAgdmFyIG1hdHJpeDEgPSBBLl9tYXRyaXg7XG4gIHZhciBtYXRyaXgyID0gQi5fbWF0cml4O1xuICByZXR1cm4gdGhpcy5nZW5lcmF0ZShyb3csIGNvbCwgZnVuY3Rpb24gKGksIGopIHtcbiAgICByZXR1cm4gbWF0cml4MVtpXVtqXSAtIG1hdHJpeDJbaV1bal07XG4gIH0pO1xufTsiLCJcInVzZSBzdHJpY3RcIjtcblxuZnVuY3Rpb24gX3NsaWNlZFRvQXJyYXkoYXJyLCBpKSB7IHJldHVybiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB8fCBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB8fCBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkoYXJyLCBpKSB8fCBfbm9uSXRlcmFibGVSZXN0KCk7IH1cblxuZnVuY3Rpb24gX25vbkl0ZXJhYmxlUmVzdCgpIHsgdGhyb3cgbmV3IFR5cGVFcnJvcihcIkludmFsaWQgYXR0ZW1wdCB0byBkZXN0cnVjdHVyZSBub24taXRlcmFibGUgaW5zdGFuY2UuXFxuSW4gb3JkZXIgdG8gYmUgaXRlcmFibGUsIG5vbi1hcnJheSBvYmplY3RzIG11c3QgaGF2ZSBhIFtTeW1ib2wuaXRlcmF0b3JdKCkgbWV0aG9kLlwiKTsgfVxuXG5mdW5jdGlvbiBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkobywgbWluTGVuKSB7IGlmICghbykgcmV0dXJuOyBpZiAodHlwZW9mIG8gPT09IFwic3RyaW5nXCIpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB2YXIgbiA9IE9iamVjdC5wcm90b3R5cGUudG9TdHJpbmcuY2FsbChvKS5zbGljZSg4LCAtMSk7IGlmIChuID09PSBcIk9iamVjdFwiICYmIG8uY29uc3RydWN0b3IpIG4gPSBvLmNvbnN0cnVjdG9yLm5hbWU7IGlmIChuID09PSBcIk1hcFwiIHx8IG4gPT09IFwiU2V0XCIpIHJldHVybiBBcnJheS5mcm9tKG8pOyBpZiAobiA9PT0gXCJBcmd1bWVudHNcIiB8fCAvXig/OlVpfEkpbnQoPzo4fDE2fDMyKSg/OkNsYW1wZWQpP0FycmF5JC8udGVzdChuKSkgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IH1cblxuZnVuY3Rpb24gX2FycmF5TGlrZVRvQXJyYXkoYXJyLCBsZW4pIHsgaWYgKGxlbiA9PSBudWxsIHx8IGxlbiA+IGFyci5sZW5ndGgpIGxlbiA9IGFyci5sZW5ndGg7IGZvciAodmFyIGkgPSAwLCBhcnIyID0gbmV3IEFycmF5KGxlbik7IGkgPCBsZW47IGkrKykgeyBhcnIyW2ldID0gYXJyW2ldOyB9IHJldHVybiBhcnIyOyB9XG5cbmZ1bmN0aW9uIF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHsgaWYgKHR5cGVvZiBTeW1ib2wgPT09IFwidW5kZWZpbmVkXCIgfHwgIShTeW1ib2wuaXRlcmF0b3IgaW4gT2JqZWN0KGFycikpKSByZXR1cm47IHZhciBfYXJyID0gW107IHZhciBfbiA9IHRydWU7IHZhciBfZCA9IGZhbHNlOyB2YXIgX2UgPSB1bmRlZmluZWQ7IHRyeSB7IGZvciAodmFyIF9pID0gYXJyW1N5bWJvbC5pdGVyYXRvcl0oKSwgX3M7ICEoX24gPSAoX3MgPSBfaS5uZXh0KCkpLmRvbmUpOyBfbiA9IHRydWUpIHsgX2Fyci5wdXNoKF9zLnZhbHVlKTsgaWYgKGkgJiYgX2Fyci5sZW5ndGggPT09IGkpIGJyZWFrOyB9IH0gY2F0Y2ggKGVycikgeyBfZCA9IHRydWU7IF9lID0gZXJyOyB9IGZpbmFsbHkgeyB0cnkgeyBpZiAoIV9uICYmIF9pW1wicmV0dXJuXCJdICE9IG51bGwpIF9pW1wicmV0dXJuXCJdKCk7IH0gZmluYWxseSB7IGlmIChfZCkgdGhyb3cgX2U7IH0gfSByZXR1cm4gX2FycjsgfVxuXG5mdW5jdGlvbiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB7IGlmIChBcnJheS5pc0FycmF5KGFycikpIHJldHVybiBhcnI7IH1cblxudmFyIF9yZXF1aXJlID0gcmVxdWlyZSgnLi4vLi4vRXJyb3InKSxcbiAgICBJTlZBTElEX01BVFJJWCA9IF9yZXF1aXJlLklOVkFMSURfTUFUUklYO1xuLyoqXHJcbiAqIEZpbmQgdGhlIHRyYW5zcG9zZSBvZiBhIG1hdHJpeC5cclxuICogQG1lbWJlcm9mIE1hdHJpeFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7IE1hdHJpeCB9IEEgLSBBbnkgTWF0cml4XHJcbiAqIEByZXR1cm5zIHsgTWF0cml4IH0gUmV0dXJucyB0cmFuc3Bvc2Ugb2YgQVxyXG4gKi9cblxuXG5mdW5jdGlvbiB0cmFuc3Bvc2UoQSkge1xuICBpZiAoIShBIGluc3RhbmNlb2YgdGhpcykpIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoSU5WQUxJRF9NQVRSSVgpO1xuICB9XG5cbiAgdmFyIF9BJHNpemUgPSBBLnNpemUoKSxcbiAgICAgIF9BJHNpemUyID0gX3NsaWNlZFRvQXJyYXkoX0Ekc2l6ZSwgMiksXG4gICAgICByb3cgPSBfQSRzaXplMlswXSxcbiAgICAgIGNvbCA9IF9BJHNpemUyWzFdO1xuXG4gIHZhciBtYXRyaXggPSBBLl9tYXRyaXg7XG4gIHJldHVybiB0aGlzLmdlbmVyYXRlKGNvbCwgcm93LCBmdW5jdGlvbiAoaSwgaikge1xuICAgIHJldHVybiBtYXRyaXhbal1baV07XG4gIH0pO1xufVxuXG47XG5tb2R1bGUuZXhwb3J0cyA9IHRyYW5zcG9zZTsiLCJcInVzZSBzdHJpY3RcIjtcblxudmFyIE1hdHJpeCA9IHJlcXVpcmUoJy4uLy4uJyk7XG5cbnZhciBfcmVxdWlyZSA9IHJlcXVpcmUoJy4uLy4uL0Vycm9yJyksXG4gICAgSU5WQUxJRF9QX05PUk0gPSBfcmVxdWlyZS5JTlZBTElEX1BfTk9STSxcbiAgICBTSU5HVUxBUl9NQVRSSVggPSBfcmVxdWlyZS5TSU5HVUxBUl9NQVRSSVgsXG4gICAgSU5WQUxJRF9TUVVBUkVfTUFUUklYID0gX3JlcXVpcmUuSU5WQUxJRF9TUVVBUkVfTUFUUklYO1xuLyoqXHJcbiAqIENhbGN1bGF0aW9ucyB0aGUgY29uZGl0aW9uIG51bWJlciBvZiBzcXVhcmUgTWF0cml4XHJcbiAqIHdpdGggcmVzcGVjdCB0byB0aGUgY2hvaWNlIG9mIE1hdHJpeCBub3JtLiBcclxuICogSWYgdGhlIE1hdHJpeCBpcyBzaW5ndWxhciwgcmV0dXJucyBJbmZpbml0eS48YnI+PGJyPlxyXG4gKiBUaGUgY29uZGl0aW9uIG51bWJlciBpcyBub3QgY2FjaGVkLlxyXG4gKiBAbWVtYmVyb2YgTWF0cml4XHJcbiAqIEBpbnN0YW5jZVxyXG4gKiBAcGFyYW0geygxfDJ8SW5maW5pdHl8J0YnKX0gcCAtIFR5cGUgb2YgTWF0cml4IG5vcm1cclxuICogQHJldHVybnMge251bWJlcn0gVGhlIGNvbmRpdGlvbiBudW1iZXIgb2YgTWF0cml4XHJcbiAqL1xuXG5cbmZ1bmN0aW9uIGNvbmQoKSB7XG4gIHZhciBwID0gYXJndW1lbnRzLmxlbmd0aCA+IDAgJiYgYXJndW1lbnRzWzBdICE9PSB1bmRlZmluZWQgPyBhcmd1bWVudHNbMF0gOiAyO1xuXG4gIGlmIChwICE9PSAxICYmIHAgIT09IDIgJiYgcCAhPT0gSW5maW5pdHkgJiYgcCAhPT0gJ0YnKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfUF9OT1JNKTtcbiAgfVxuXG4gIGlmICghdGhpcy5pc1NxdWFyZSgpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfU1FVQVJFX01BVFJJWCk7XG4gIH1cblxuICB0cnkge1xuICAgIHZhciBpbnZlcnNlID0gTWF0cml4LmludmVyc2UodGhpcyk7XG4gICAgcmV0dXJuIGludmVyc2Uubm9ybShwKSAqIHRoaXMubm9ybShwKTtcbiAgfSBjYXRjaCAoZXJyb3IpIHtcbiAgICBpZiAoZXJyb3IubWVzc2FnZSA9PT0gU0lOR1VMQVJfTUFUUklYKSB7XG4gICAgICByZXR1cm4gSW5maW5pdHk7XG4gICAgfVxuXG4gICAgdGhyb3cgZXJyb3I7XG4gIH1cbn1cblxuO1xubW9kdWxlLmV4cG9ydHMgPSBjb25kOyIsIlwidXNlIHN0cmljdFwiO1xuXG5mdW5jdGlvbiBfc2xpY2VkVG9BcnJheShhcnIsIGkpIHsgcmV0dXJuIF9hcnJheVdpdGhIb2xlcyhhcnIpIHx8IF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHx8IF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShhcnIsIGkpIHx8IF9ub25JdGVyYWJsZVJlc3QoKTsgfVxuXG5mdW5jdGlvbiBfbm9uSXRlcmFibGVSZXN0KCkgeyB0aHJvdyBuZXcgVHlwZUVycm9yKFwiSW52YWxpZCBhdHRlbXB0IHRvIGRlc3RydWN0dXJlIG5vbi1pdGVyYWJsZSBpbnN0YW5jZS5cXG5JbiBvcmRlciB0byBiZSBpdGVyYWJsZSwgbm9uLWFycmF5IG9iamVjdHMgbXVzdCBoYXZlIGEgW1N5bWJvbC5pdGVyYXRvcl0oKSBtZXRob2QuXCIpOyB9XG5cbmZ1bmN0aW9uIF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShvLCBtaW5MZW4pIHsgaWYgKCFvKSByZXR1cm47IGlmICh0eXBlb2YgbyA9PT0gXCJzdHJpbmdcIikgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IHZhciBuID0gT2JqZWN0LnByb3RvdHlwZS50b1N0cmluZy5jYWxsKG8pLnNsaWNlKDgsIC0xKTsgaWYgKG4gPT09IFwiT2JqZWN0XCIgJiYgby5jb25zdHJ1Y3RvcikgbiA9IG8uY29uc3RydWN0b3IubmFtZTsgaWYgKG4gPT09IFwiTWFwXCIgfHwgbiA9PT0gXCJTZXRcIikgcmV0dXJuIEFycmF5LmZyb20obyk7IGlmIChuID09PSBcIkFyZ3VtZW50c1wiIHx8IC9eKD86VWl8SSludCg/Ojh8MTZ8MzIpKD86Q2xhbXBlZCk/QXJyYXkkLy50ZXN0KG4pKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgfVxuXG5mdW5jdGlvbiBfYXJyYXlMaWtlVG9BcnJheShhcnIsIGxlbikgeyBpZiAobGVuID09IG51bGwgfHwgbGVuID4gYXJyLmxlbmd0aCkgbGVuID0gYXJyLmxlbmd0aDsgZm9yICh2YXIgaSA9IDAsIGFycjIgPSBuZXcgQXJyYXkobGVuKTsgaSA8IGxlbjsgaSsrKSB7IGFycjJbaV0gPSBhcnJbaV07IH0gcmV0dXJuIGFycjI7IH1cblxuZnVuY3Rpb24gX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgeyBpZiAodHlwZW9mIFN5bWJvbCA9PT0gXCJ1bmRlZmluZWRcIiB8fCAhKFN5bWJvbC5pdGVyYXRvciBpbiBPYmplY3QoYXJyKSkpIHJldHVybjsgdmFyIF9hcnIgPSBbXTsgdmFyIF9uID0gdHJ1ZTsgdmFyIF9kID0gZmFsc2U7IHZhciBfZSA9IHVuZGVmaW5lZDsgdHJ5IHsgZm9yICh2YXIgX2kgPSBhcnJbU3ltYm9sLml0ZXJhdG9yXSgpLCBfczsgIShfbiA9IChfcyA9IF9pLm5leHQoKSkuZG9uZSk7IF9uID0gdHJ1ZSkgeyBfYXJyLnB1c2goX3MudmFsdWUpOyBpZiAoaSAmJiBfYXJyLmxlbmd0aCA9PT0gaSkgYnJlYWs7IH0gfSBjYXRjaCAoZXJyKSB7IF9kID0gdHJ1ZTsgX2UgPSBlcnI7IH0gZmluYWxseSB7IHRyeSB7IGlmICghX24gJiYgX2lbXCJyZXR1cm5cIl0gIT0gbnVsbCkgX2lbXCJyZXR1cm5cIl0oKTsgfSBmaW5hbGx5IHsgaWYgKF9kKSB0aHJvdyBfZTsgfSB9IHJldHVybiBfYXJyOyB9XG5cbmZ1bmN0aW9uIF9hcnJheVdpdGhIb2xlcyhhcnIpIHsgaWYgKEFycmF5LmlzQXJyYXkoYXJyKSkgcmV0dXJuIGFycjsgfVxuXG4vKiBlc2xpbnQtZGlzYWJsZSBwcmVmZXItZGVzdHJ1Y3R1cmluZyAqL1xudmFyIE1hdHJpeCA9IHJlcXVpcmUoJy4uLy4uJyk7XG5cbnZhciBfcmVxdWlyZSA9IHJlcXVpcmUoJy4uLy4uL0Vycm9yJyksXG4gICAgSU5WQUxJRF9TUVVBUkVfTUFUUklYID0gX3JlcXVpcmUuSU5WQUxJRF9TUVVBUkVfTUFUUklYO1xuLyoqXHJcbiAqIENhbGN1bGF0ZXMgdGhlIGRldGVybWluYW50IG9mIHNxdWFyZSBNYXRyaXguXHJcbiAqIElmIHRoZSBNYXRyaXggc2l6ZSBpcyBsYXJnZXIgdGhhbiAzLCBpdCBjYWxjdWxhdGVzIHRoZSBkZXRlcm1pbmFudCB1c2luZ1xyXG4gKiBMVSBkZWNvbXBvc2l0aW9uLCBvdGhlcndpc2UsIHVzaW5nIExlaWJuaXogRm9ybXVsYS48YnI+PGJyPlxyXG4gKiBUaGUgZGV0ZXJtaW5hbnQgaXMgY2FjaGVkLlxyXG4gKiBAbWVtYmVyb2YgTWF0cml4XHJcbiAqIEBpbnN0YW5jZVxyXG4gKiBAcmV0dXJucyB7bnVtYmVyfSBSZXR1cm5zIHRoZSBkZXRlcm1pbmFudCBvZiBzcXVhcmUgbWF0cmlyeFxyXG4gKi9cblxuXG5mdW5jdGlvbiBkZXQoKSB7XG4gIGlmICghdGhpcy5pc1NxdWFyZSgpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfU1FVQVJFX01BVFJJWCk7XG4gIH1cblxuICBpZiAodGhpcy5fZGV0ICE9PSB1bmRlZmluZWQpIHtcbiAgICByZXR1cm4gdGhpcy5fZGV0O1xuICB9XG5cbiAgdmFyIG1hdHJpeCA9IHRoaXMuX21hdHJpeDtcbiAgdmFyIHNpemUgPSBtYXRyaXgubGVuZ3RoO1xuXG4gIGlmIChzaXplID09PSAwKSB7XG4gICAgdGhpcy5fZGV0ID0gMTtcbiAgICByZXR1cm4gMTsgLy8gdGhlIGRldGVybWluYW50IG9mIDB4MCBtYXRyaXggbXVzdCBiZSAxXG4gIH1cblxuICBpZiAoc2l6ZSA9PT0gMSkge1xuICAgIHRoaXMuX2RldCA9IG1hdHJpeFswXVswXTtcbiAgICByZXR1cm4gdGhpcy5fZGV0O1xuICB9XG5cbiAgaWYgKHNpemUgPT09IDIpIHtcbiAgICB0aGlzLl9kZXQgPSBtYXRyaXhbMF1bMF0gKiBtYXRyaXhbMV1bMV0gLSBtYXRyaXhbMF1bMV0gKiBtYXRyaXhbMV1bMF07XG4gICAgcmV0dXJuIHRoaXMuX2RldDtcbiAgfVxuXG4gIGlmIChzaXplID09PSAzKSB7XG4gICAgdGhpcy5fZGV0ID0gbWF0cml4WzBdWzBdICogbWF0cml4WzFdWzFdICogbWF0cml4WzJdWzJdICsgbWF0cml4WzBdWzFdICogbWF0cml4WzFdWzJdICogbWF0cml4WzJdWzBdICsgbWF0cml4WzBdWzJdICogbWF0cml4WzFdWzBdICogbWF0cml4WzJdWzFdIC0gbWF0cml4WzBdWzJdICogbWF0cml4WzFdWzFdICogbWF0cml4WzJdWzBdIC0gbWF0cml4WzBdWzFdICogbWF0cml4WzFdWzBdICogbWF0cml4WzJdWzJdIC0gbWF0cml4WzBdWzBdICogbWF0cml4WzFdWzJdICogbWF0cml4WzJdWzFdO1xuICAgIHJldHVybiB0aGlzLl9kZXQ7XG4gIH1cblxuICB2YXIgX01hdHJpeCRMVSA9IE1hdHJpeC5MVSh0aGlzLCB0cnVlKSxcbiAgICAgIF9NYXRyaXgkTFUyID0gX3NsaWNlZFRvQXJyYXkoX01hdHJpeCRMVSwgMiksXG4gICAgICBQID0gX01hdHJpeCRMVTJbMF0sXG4gICAgICBMVSA9IF9NYXRyaXgkTFUyWzFdO1xuXG4gIHZhciBtYXRyaXhMVSA9IExVLl9tYXRyaXg7IC8vIGNvdW50IHdoZXRoZXIgdGhlIG51bWJlciBvZiBwZXJtdXRhdGlvbnMgPHN3YXA+IGlzIG9kZCBvciBldmVuXG4gIC8vIE8obl4yKVxuXG4gIHZhciBzd2FwID0gMDtcblxuICBmb3IgKHZhciBpID0gMDsgaSA8IHNpemU7IGkrKykge1xuICAgIGlmIChQW2ldID09PSBpKSB7XG4gICAgICBjb250aW51ZTtcbiAgICB9XG5cbiAgICB3aGlsZSAoUFtpXSAhPT0gaSkge1xuICAgICAgdmFyIHRhcmdldCA9IFBbaV07XG4gICAgICBQW2ldID0gUFt0YXJnZXRdO1xuICAgICAgUFt0YXJnZXRdID0gdGFyZ2V0O1xuICAgICAgc3dhcCsrO1xuICAgIH1cbiAgfVxuXG4gIHZhciByZXN1bHQgPSAxO1xuXG4gIGZvciAodmFyIF9pMiA9IDA7IF9pMiA8IHNpemU7IF9pMisrKSB7XG4gICAgcmVzdWx0ICo9IG1hdHJpeExVW19pMl1bX2kyXTtcbiAgfVxuXG4gIGlmIChzd2FwICUgMiA9PT0gMSkge1xuICAgIHRoaXMuX2RldCA9IHJlc3VsdCAqIC0xO1xuICAgIHJldHVybiB0aGlzLl9kZXQ7XG4gIH1cblxuICB0aGlzLl9kZXQgPSByZXN1bHQ7XG4gIHJldHVybiByZXN1bHQ7XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gZGV0OyIsIlwidXNlIHN0cmljdFwiO1xuXG5mdW5jdGlvbiBfc2xpY2VkVG9BcnJheShhcnIsIGkpIHsgcmV0dXJuIF9hcnJheVdpdGhIb2xlcyhhcnIpIHx8IF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHx8IF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShhcnIsIGkpIHx8IF9ub25JdGVyYWJsZVJlc3QoKTsgfVxuXG5mdW5jdGlvbiBfbm9uSXRlcmFibGVSZXN0KCkgeyB0aHJvdyBuZXcgVHlwZUVycm9yKFwiSW52YWxpZCBhdHRlbXB0IHRvIGRlc3RydWN0dXJlIG5vbi1pdGVyYWJsZSBpbnN0YW5jZS5cXG5JbiBvcmRlciB0byBiZSBpdGVyYWJsZSwgbm9uLWFycmF5IG9iamVjdHMgbXVzdCBoYXZlIGEgW1N5bWJvbC5pdGVyYXRvcl0oKSBtZXRob2QuXCIpOyB9XG5cbmZ1bmN0aW9uIF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShvLCBtaW5MZW4pIHsgaWYgKCFvKSByZXR1cm47IGlmICh0eXBlb2YgbyA9PT0gXCJzdHJpbmdcIikgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IHZhciBuID0gT2JqZWN0LnByb3RvdHlwZS50b1N0cmluZy5jYWxsKG8pLnNsaWNlKDgsIC0xKTsgaWYgKG4gPT09IFwiT2JqZWN0XCIgJiYgby5jb25zdHJ1Y3RvcikgbiA9IG8uY29uc3RydWN0b3IubmFtZTsgaWYgKG4gPT09IFwiTWFwXCIgfHwgbiA9PT0gXCJTZXRcIikgcmV0dXJuIEFycmF5LmZyb20obyk7IGlmIChuID09PSBcIkFyZ3VtZW50c1wiIHx8IC9eKD86VWl8SSludCg/Ojh8MTZ8MzIpKD86Q2xhbXBlZCk/QXJyYXkkLy50ZXN0KG4pKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgfVxuXG5mdW5jdGlvbiBfYXJyYXlMaWtlVG9BcnJheShhcnIsIGxlbikgeyBpZiAobGVuID09IG51bGwgfHwgbGVuID4gYXJyLmxlbmd0aCkgbGVuID0gYXJyLmxlbmd0aDsgZm9yICh2YXIgaSA9IDAsIGFycjIgPSBuZXcgQXJyYXkobGVuKTsgaSA8IGxlbjsgaSsrKSB7IGFycjJbaV0gPSBhcnJbaV07IH0gcmV0dXJuIGFycjI7IH1cblxuZnVuY3Rpb24gX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgeyBpZiAodHlwZW9mIFN5bWJvbCA9PT0gXCJ1bmRlZmluZWRcIiB8fCAhKFN5bWJvbC5pdGVyYXRvciBpbiBPYmplY3QoYXJyKSkpIHJldHVybjsgdmFyIF9hcnIgPSBbXTsgdmFyIF9uID0gdHJ1ZTsgdmFyIF9kID0gZmFsc2U7IHZhciBfZSA9IHVuZGVmaW5lZDsgdHJ5IHsgZm9yICh2YXIgX2kgPSBhcnJbU3ltYm9sLml0ZXJhdG9yXSgpLCBfczsgIShfbiA9IChfcyA9IF9pLm5leHQoKSkuZG9uZSk7IF9uID0gdHJ1ZSkgeyBfYXJyLnB1c2goX3MudmFsdWUpOyBpZiAoaSAmJiBfYXJyLmxlbmd0aCA9PT0gaSkgYnJlYWs7IH0gfSBjYXRjaCAoZXJyKSB7IF9kID0gdHJ1ZTsgX2UgPSBlcnI7IH0gZmluYWxseSB7IHRyeSB7IGlmICghX24gJiYgX2lbXCJyZXR1cm5cIl0gIT0gbnVsbCkgX2lbXCJyZXR1cm5cIl0oKTsgfSBmaW5hbGx5IHsgaWYgKF9kKSB0aHJvdyBfZTsgfSB9IHJldHVybiBfYXJyOyB9XG5cbmZ1bmN0aW9uIF9hcnJheVdpdGhIb2xlcyhhcnIpIHsgaWYgKEFycmF5LmlzQXJyYXkoYXJyKSkgcmV0dXJuIGFycjsgfVxuXG4vKiBlc2xpbnQtZGlzYWJsZSBuby1wYXJhbS1yZWFzc2lnbiAqL1xuLy8gcmVmZXJlbmNlOiBodHRwczovL3Blb3BsZS5pbmYuZXRoei5jaC9hcmJlbnovZXdwL0xub3Rlcy9jaGFwdGVyNC5wZGZcbnZhciBDb21wbGV4ID0gcmVxdWlyZSgnQHJheXlhbWhrL2NvbXBsZXgnKTtcblxudmFyIE1hdHJpeCA9IHJlcXVpcmUoJy4uLy4uJyk7XG5cbnZhciBfcmVxdWlyZSA9IHJlcXVpcmUoJy4uLy4uL0Vycm9yJyksXG4gICAgSU5WQUxJRF9TUVVBUkVfTUFUUklYID0gX3JlcXVpcmUuSU5WQUxJRF9TUVVBUkVfTUFUUklYO1xuLyoqXHJcbiAqIENhbGN1bGF0ZXMgdGhlIGVpZ2VudmFsdWVzIG9mIGFueSBzcXVhcmUgTWF0cml4IHVzaW5nIFFSIEFsZ29yaXRobS48YnI+PGJyPlxyXG4gKiBcclxuICogVGhlIGVpZ2VudmFsdWVzIGNhbiBiZSBlaXRoZXIgcmVhbCBudW1iZXIgb3IgY29tcGxleCBudW1iZXIuXHJcbiAqIE5vdGUgdGhhdCBhbGwgZWlnZW52YWx1ZXMgYXJlIGluc3RhbmNlIG9mIENvbXBsZXgsXHJcbiAqIGZvciBtb3JlIGRldGFpbHMgcGxlYXNlIHZpc2l0IFtDb21wbGV4LmpzXXtAbGluayBodHRwczovL2dpdGh1Yi5jb20vcmF5eWFtaGsvQ29tcGxleC5qc30uPGJyPjxicj5cclxuICogXHJcbiAqIFRoZSBlaWdlbnZhbHVlcyBhcmUgY2FjaGVkLlxyXG4gKiBAbWVtYmVyb2YgTWF0cml4XHJcbiAqIEBpbnN0YW5jZVxyXG4gKiBAcmV0dXJucyB7Q29tcGxleFtdfSBBcnJheSBvZiBlaWdlbnZhbHVlc1xyXG4gKi9cblxuXG5mdW5jdGlvbiBlaWdlbnZhbHVlcygpIHtcbiAgaWYgKCF0aGlzLmlzU3F1YXJlKCkpIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoSU5WQUxJRF9TUVVBUkVfTUFUUklYKTtcbiAgfVxuXG4gIGlmICh0aGlzLl9laWdlbnZhbHVlcyAhPT0gdW5kZWZpbmVkKSB7XG4gICAgcmV0dXJuIHRoaXMuX2VpZ2VudmFsdWVzO1xuICB9XG5cbiAgdmFyIHNpemUgPSB0aGlzLnNpemUoKVswXTtcbiAgdmFyIHZhbHVlcyA9IFtdO1xuICB2YXIgZGlnaXQgPSB0aGlzLl9kaWdpdDtcbiAgdmFyIEVQU0lMT04gPSAxIC8gKE1hdGgucG93KDEwLCBkaWdpdCkgKiAyKTtcblxuICB2YXIgY2xvbmUgPSBNYXRyaXguY2xvbmUodGhpcykuX21hdHJpeDtcblxuICB2YXIgaXNDb252ZXJnZW50ID0gdHJ1ZTsgLy8gZmxhZ1xuXG4gIHZhciBza2lwID0gZmFsc2U7IC8vIFRyYW5zZm9ybSBtYXRyaXggdG8gSGVzc2VuYmVyZyBtYXRyaXhcblxuICBIb3VzZWhvbGRlclRyYW5zZm9ybShjbG9uZSwgZGlnaXQpO1xuXG4gIGZvciAodmFyIGkgPSBzaXplIC0gMTsgaSA+IDA7IGktLSkge1xuICAgIHZhciBkaXZlcmdlbmNlQ291bnQgPSAwO1xuICAgIHZhciBwcmV2ID0gdm9pZCAwOyAvLyB1c2VkIHRvIGRldGVybWluZSBjb252ZXJnZW5jZVxuICAgIC8vIGlmIG9idGFpbnMgY29tcGxleCBlaWdlbnZhbHVlcyBwYWlyIGluIHByZXZpb3VzIGl0ZXJhdGlvbiwgc2tpcCBjdXJyZW50IHJvdW5kXG5cbiAgICBpZiAoc2tpcCkge1xuICAgICAgc2tpcCA9IGZhbHNlO1xuICAgICAgY29udGludWU7XG4gICAgfVxuXG4gICAgdmFyIHNoaWZ0ID0gY2xvbmVbc2l6ZSAtIDFdW3NpemUgLSAxXTsgLy8gZXNsaW50LWRpc2FibGUtbmV4dC1saW5lIG5vLWNvbnN0YW50LWNvbmRpdGlvblxuXG4gICAgd2hpbGUgKHRydWUpIHtcbiAgICAgIGlmICghaXNDb252ZXJnZW50KSB7XG4gICAgICAgIC8vIGlmIHRoZSBjdXJyZW50IGVpZ2VudmFsdWUgaXMgbm90IHJlYWxcbiAgICAgICAgcHJldiA9IHNpemUyRWlnZW52YWx1ZXMoY2xvbmVbaSAtIDFdW2kgLSAxXSwgY2xvbmVbaSAtIDFdW2ldLCBjbG9uZVtpXVtpIC0gMV0sIGNsb25lW2ldW2ldKS5tZXRyaWM7XG4gICAgICB9IGVsc2Uge1xuICAgICAgICAvLyBpZiB0aGUgY3VycmVudCBlaWdlbnZhbHVlIGlzIHJlYWxcbiAgICAgICAgcHJldiA9IE1hdGguYWJzKGNsb25lW2ldW2kgLSAxXSk7XG4gICAgICB9IC8vIGFwcGx5IHNpbmdsZSBzaGlmdFxuXG5cbiAgICAgIGZvciAodmFyIGogPSAwOyBqIDwgc2l6ZTsgaisrKSB7XG4gICAgICAgIGNsb25lW2pdW2pdIC09IHNoaWZ0O1xuICAgICAgfSAvLyBBcHBseSBRUiBBbGdvcml0aG1cblxuXG4gICAgICBIZXNzZW5iZXJnUVIoY2xvbmUsIGRpZ2l0KTtcblxuICAgICAgZm9yICh2YXIgX2ogPSAwOyBfaiA8IHNpemU7IF9qKyspIHtcbiAgICAgICAgY2xvbmVbX2pdW19qXSArPSBzaGlmdDtcbiAgICAgIH1cblxuICAgICAgaWYgKGlzQ29udmVyZ2VudCAmJiBwcmV2IDwgTWF0aC5hYnMoY2xvbmVbaV1baSAtIDFdKSkge1xuICAgICAgICBkaXZlcmdlbmNlQ291bnQrKztcbiAgICAgIH0gLy8gaWYgdGhlIGN1cnJlbnQgZWlnZW52YWx1ZSBpcyByZWFsIGFuZCB0aGUgZW50cnkgaXMgYWxtb3N0IFpFUk8gPT4gYnJlYWs7XG5cblxuICAgICAgaWYgKGlzQ29udmVyZ2VudCAmJiBNYXRoLmFicyhjbG9uZVtpXVtpIC0gMV0pIDwgRVBTSUxPTikge1xuICAgICAgICB2YWx1ZXNbaV0gPSBuZXcgQ29tcGxleChjbG9uZVtpXVtpXSk7XG4gICAgICAgIGJyZWFrO1xuICAgICAgfSAvLyBpZiB0aGUgY3VycmVudCBlaWdlbnZhbHVlcyBwYWlyIGlzIGNvbXBsZXgsIGlmIHRoZSBkaWZmZXJlbmNlIG9mIHRoZSBwcmV2aW91cyBlaWdhbnZhbHVlcyBhbmQgdGhlXG4gICAgICAvLyBlaWdlbnZhbHVlcyBvZiBzdWJtYXRyaXggaXMgYWxtb3N0IFpFUk8gPT4gYnJlYWtcblxuXG4gICAgICB2YXIgX3NpemUyRWlnZW52YWx1ZXMgPSBzaXplMkVpZ2VudmFsdWVzKGNsb25lW2kgLSAxXVtpIC0gMV0sIGNsb25lW2kgLSAxXVtpXSwgY2xvbmVbaV1baSAtIDFdLCBjbG9uZVtpXVtpXSksXG4gICAgICAgICAgbWV0cmljID0gX3NpemUyRWlnZW52YWx1ZXMubWV0cmljLFxuICAgICAgICAgIGVpZ2VuMSA9IF9zaXplMkVpZ2VudmFsdWVzLmVpZ2VuMSxcbiAgICAgICAgICBlaWdlbjIgPSBfc2l6ZTJFaWdlbnZhbHVlcy5laWdlbjI7XG5cbiAgICAgIGlmICghaXNDb252ZXJnZW50ICYmIE1hdGguYWJzKHByZXYgLSBtZXRyaWMpIDwgRVBTSUxPTikge1xuICAgICAgICBpc0NvbnZlcmdlbnQgPSB0cnVlOyAvLyByZS1pbml0aWFsaXplXG5cbiAgICAgICAgc2tpcCA9IHRydWU7XG4gICAgICAgIHZhciByZTEgPSBlaWdlbjEucmUsXG4gICAgICAgICAgICBpbTEgPSBlaWdlbjEuaW07XG4gICAgICAgIHZhciByZTIgPSBlaWdlbjIucmUsXG4gICAgICAgICAgICBpbTIgPSBlaWdlbjIuaW07XG4gICAgICAgIHZhbHVlc1tpXSA9IG5ldyBDb21wbGV4KHJlMSwgaW0xKTtcbiAgICAgICAgdmFsdWVzW2kgLSAxXSA9IG5ldyBDb21wbGV4KHJlMiwgaW0yKTtcbiAgICAgICAgYnJlYWs7XG4gICAgICB9IC8vIGlmIHRoZSBlbnRyeSBkb2Vzbid0IGNvbnZlcmdlID0+IGNvbXBsZXggZWlnZW52YWx1ZXMgcGFpclxuXG5cbiAgICAgIGlmIChkaXZlcmdlbmNlQ291bnQgPiAzKSB7XG4gICAgICAgIGlzQ29udmVyZ2VudCA9IGZhbHNlO1xuICAgICAgfVxuICAgIH1cbiAgfVxuXG4gIGlmICghc2tpcCkge1xuICAgIHZhbHVlc1swXSA9IG5ldyBDb21wbGV4KGNsb25lWzBdWzBdKTtcbiAgfVxuXG4gIHRoaXMuX2VpZ2VudmFsdWVzID0gdmFsdWVzO1xuICByZXR1cm4gdmFsdWVzO1xufVxuXG47XG5cbmZ1bmN0aW9uIEhvdXNlaG9sZGVyVHJhbnNmb3JtKEEsIGRpZ2l0KSB7XG4gIHZhciBzaXplID0gQS5sZW5ndGg7XG4gIHZhciBFUFNJTE9OID0gMSAvIChNYXRoLnBvdygxMCwgZGlnaXQpICogMik7XG5cbiAgZm9yICh2YXIgaiA9IDA7IGogPCBzaXplIC0gMjsgaisrKSB7XG4gICAgdmFyIHhOb3JtID0gMDtcbiAgICB2YXIgdSA9IG5ldyBBcnJheShzaXplIC0gaiAtIDEpO1xuXG4gICAgZm9yICh2YXIgaSA9IGogKyAxOyBpIDwgc2l6ZTsgaSsrKSB7XG4gICAgICB2YXIgZW50cnkgPSBBW2ldW2pdO1xuICAgICAgeE5vcm0gKz0gTWF0aC5wb3coZW50cnksIDIpO1xuICAgICAgdVtpIC0gaiAtIDFdID0gZW50cnk7XG4gICAgfVxuXG4gICAgeE5vcm0gPSBNYXRoLnNxcnQoeE5vcm0pO1xuXG4gICAgaWYgKE1hdGguYWJzKHhOb3JtKSA8IEVQU0lMT04pIHtcbiAgICAgIGNvbnRpbnVlO1xuICAgIH1cblxuICAgIGlmICh1WzBdID49IDApIHtcbiAgICAgIHVbMF0gKz0geE5vcm07XG4gICAgfSBlbHNlIHtcbiAgICAgIHVbMF0gLT0geE5vcm07XG4gICAgfSAvLyBNYWtlICd1JyB1bml0IHZlY3RvclxuXG5cbiAgICB2YXIgdU5vcm0gPSAwO1xuXG4gICAgZm9yICh2YXIgX2kgPSAwOyBfaSA8IHUubGVuZ3RoOyBfaSsrKSB7XG4gICAgICB1Tm9ybSArPSBNYXRoLnBvdyh1W19pXSwgMik7XG4gICAgfVxuXG4gICAgdU5vcm0gPSBNYXRoLnNxcnQodU5vcm0pO1xuXG4gICAgZm9yICh2YXIgX2kyID0gMDsgX2kyIDwgdS5sZW5ndGg7IF9pMisrKSB7XG4gICAgICB1W19pMl0gLz0gdU5vcm07XG4gICAgfSAvLyB1cGRhdGUgdGhlIG1hdHJpeCwgbXVsdGlwbHkgUCBmcm9tIGxlZnRcblxuXG4gICAgZm9yICh2YXIgbiA9IGo7IG4gPCBzaXplOyBuKyspIHtcbiAgICAgIC8vIGNvbHVtblxuICAgICAgdmFyIHYgPSBuZXcgQXJyYXkoc2l6ZSAtIGogLSAxKTtcblxuICAgICAgZm9yICh2YXIgbSA9IGogKyAxOyBtIDwgc2l6ZTsgbSsrKSB7XG4gICAgICAgIHZbbSAtIGogLSAxXSA9IEFbbV1bbl07XG4gICAgICB9XG5cbiAgICAgIHZhciBzY2FsZXIgPSAwO1xuXG4gICAgICBmb3IgKHZhciBfbSA9IDA7IF9tIDwgdi5sZW5ndGg7IF9tKyspIHtcbiAgICAgICAgc2NhbGVyICs9IHZbX21dICogdVtfbV07XG4gICAgICB9XG5cbiAgICAgIHNjYWxlciAqPSAyO1xuXG4gICAgICBmb3IgKHZhciBfbTIgPSBqICsgMTsgX20yIDwgc2l6ZTsgX20yKyspIHtcbiAgICAgICAgLy8gcm93XG4gICAgICAgIGlmIChuID09PSBqICYmIF9tMiAhPT0gaiArIDEpIHtcbiAgICAgICAgICBBW19tMl1bbl0gPSAwO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgIEFbX20yXVtuXSA9IHZbX20yIC0gaiAtIDFdIC0gc2NhbGVyICogdVtfbTIgLSBqIC0gMV07XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICB9IC8vIHVwZGF0ZSB0aGUgbWF0cml4LCBtdWx0aXBseSBQIGZyb20gcmlnaHRcblxuXG4gICAgZm9yICh2YXIgX20zID0gMDsgX20zIDwgc2l6ZTsgX20zKyspIHtcbiAgICAgIC8vIHJvd1xuICAgICAgdmFyIF92ID0gbmV3IEFycmF5KHNpemUgLSBqIC0gMSk7XG5cbiAgICAgIGZvciAodmFyIF9uID0gaiArIDE7IF9uIDwgc2l6ZTsgX24rKykge1xuICAgICAgICBfdltfbiAtIGogLSAxXSA9IEFbX20zXVtfbl07XG4gICAgICB9XG5cbiAgICAgIHZhciBfc2NhbGVyID0gMDtcblxuICAgICAgZm9yICh2YXIgX24yID0gMDsgX24yIDwgX3YubGVuZ3RoOyBfbjIrKykge1xuICAgICAgICBfc2NhbGVyICs9IF92W19uMl0gKiB1W19uMl07XG4gICAgICB9XG5cbiAgICAgIF9zY2FsZXIgKj0gMjtcblxuICAgICAgZm9yICh2YXIgX24zID0gaiArIDE7IF9uMyA8IHNpemU7IF9uMysrKSB7XG4gICAgICAgIC8vIGNvbHVtblxuICAgICAgICBBW19tM11bX24zXSA9IF92W19uMyAtIGogLSAxXSAtIF9zY2FsZXIgKiB1W19uMyAtIGogLSAxXTtcbiAgICAgIH1cbiAgICB9XG4gIH1cbn1cblxuZnVuY3Rpb24gSGVzc2VuYmVyZ1FSKEgsIGRpZ2l0KSB7XG4gIHZhciBzaXplID0gSC5sZW5ndGg7XG4gIHZhciBFUFNJTE9OID0gMSAvIChNYXRoLnBvdygxMCwgZGlnaXQpICogMik7XG4gIHZhciBzaW5jb3MgPSBuZXcgQXJyYXkoc2l6ZSAtIDEpO1xuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgc2l6ZSAtIDE7IGkrKykge1xuICAgIHZhciBhID0gSFtpXVtpXTtcbiAgICB2YXIgYyA9IEhbaSArIDFdW2ldO1xuICAgIHZhciBub3JtID0gTWF0aC5zcXJ0KE1hdGgucG93KGEsIDIpICsgTWF0aC5wb3coYywgMikpO1xuXG4gICAgaWYgKG5vcm0gPCBFUFNJTE9OKSB7XG4gICAgICBjb250aW51ZTtcbiAgICB9XG5cbiAgICB2YXIgY29zID0gYSAvIG5vcm07XG4gICAgdmFyIHNpbiA9IGMgKiAtMSAvIG5vcm07XG4gICAgc2luY29zW2ldID0gW3NpbiwgY29zXTtcbiAgICB2YXIgcm93MSA9IG5ldyBBcnJheShzaXplIC0gaSk7XG4gICAgdmFyIHJvdzIgPSBuZXcgQXJyYXkoc2l6ZSAtIGkpO1xuXG4gICAgZm9yICh2YXIgaiA9IGk7IGogPCBzaXplOyBqKyspIHtcbiAgICAgIHJvdzFbaiAtIGldID0gSFtpXVtqXTtcbiAgICAgIHJvdzJbaiAtIGldID0gSFtpICsgMV1bal07XG4gICAgfVxuXG4gICAgZm9yICh2YXIgX2oyID0gaTsgX2oyIDwgc2l6ZTsgX2oyKyspIHtcbiAgICAgIEhbaV1bX2oyXSA9IGNvcyAqIHJvdzFbX2oyIC0gaV0gKyBzaW4gKiAtMSAqIHJvdzJbX2oyIC0gaV07XG5cbiAgICAgIGlmIChpID09PSBfajIpIHtcbiAgICAgICAgSFtpICsgMV1bX2oyXSA9IDA7XG4gICAgICB9IGVsc2Uge1xuICAgICAgICBIW2kgKyAxXVtfajJdID0gc2luICogcm93MVtfajIgLSBpXSArIGNvcyAqIHJvdzJbX2oyIC0gaV07XG4gICAgICB9XG4gICAgfVxuICB9XG5cbiAgZm9yICh2YXIgX2ozID0gMDsgX2ozIDwgc2l6ZSAtIDE7IF9qMysrKSB7XG4gICAgaWYgKCFzaW5jb3NbX2ozXSkge1xuICAgICAgY29udGludWU7XG4gICAgfVxuXG4gICAgdmFyIF9zaW5jb3MkX2ogPSBfc2xpY2VkVG9BcnJheShzaW5jb3NbX2ozXSwgMiksXG4gICAgICAgIF9zaW4gPSBfc2luY29zJF9qWzBdLFxuICAgICAgICBfY29zID0gX3NpbmNvcyRfalsxXTtcblxuICAgIHZhciBjb2wxID0gbmV3IEFycmF5KF9qMyArIDIpO1xuICAgIHZhciBjb2wyID0gbmV3IEFycmF5KF9qMyArIDIpO1xuXG4gICAgZm9yICh2YXIgX2kzID0gMDsgX2kzIDw9IF9qMyArIDE7IF9pMysrKSB7XG4gICAgICBjb2wxW19pM10gPSBIW19pM11bX2ozXTtcbiAgICAgIGNvbDJbX2kzXSA9IEhbX2kzXVtfajMgKyAxXTtcbiAgICB9XG5cbiAgICBmb3IgKHZhciBfaTQgPSAwOyBfaTQgPD0gX2ozICsgMTsgX2k0KyspIHtcbiAgICAgIEhbX2k0XVtfajNdID0gY29sMVtfaTRdICogX2NvcyAtIGNvbDJbX2k0XSAqIF9zaW47XG4gICAgICBIW19pNF1bX2ozICsgMV0gPSBjb2wxW19pNF0gKiBfc2luICsgY29sMltfaTRdICogX2NvcztcbiAgICB9XG4gIH1cbn0gLy8gZmluZCB0aGUgZWlnZW52YWx1ZXMgb2YgMngyIG1hdHJpeFxuXG5cbmZ1bmN0aW9uIHNpemUyRWlnZW52YWx1ZXMoZTExLCBlMTIsIGUyMSwgZTIyKSB7XG4gIHZhciBiID0gKGUxMSArIGUyMikgKiAtMTtcbiAgdmFyIGMgPSBlMTEgKiBlMjIgLSBlMjEgKiBlMTI7XG4gIHZhciBkZWx0YSA9IE1hdGgucG93KGIsIDIpIC0gNCAqIGM7XG4gIHZhciByZTE7XG4gIHZhciBpbTE7XG4gIHZhciByZTI7XG4gIHZhciBpbTI7XG5cbiAgaWYgKGRlbHRhID49IDApIHtcbiAgICBpbTEgPSAwO1xuICAgIGltMiA9IDA7XG5cbiAgICBpZiAoYiA+PSAwKSB7XG4gICAgICByZTEgPSAoYiAqIC0xIC0gTWF0aC5zcXJ0KGRlbHRhKSkgLyAyO1xuICAgIH0gZWxzZSB7XG4gICAgICByZTEgPSAoYiAqIC0xICsgTWF0aC5zcXJ0KGRlbHRhKSkgLyAyO1xuICAgIH1cblxuICAgIHJlMiA9IGMgLyByZTE7XG4gIH0gZWxzZSB7XG4gICAgcmUxID0gLWIgLyAyO1xuICAgIHJlMiA9IHJlMTtcbiAgICBpbTEgPSBNYXRoLnNxcnQoZGVsdGEgKiAtMSkgLyAyO1xuICAgIGltMiA9IGltMSAqIC0xO1xuICB9XG5cbiAgcmV0dXJuIHtcbiAgICBtZXRyaWM6IE1hdGguc3FydChNYXRoLnBvdyhyZTEsIDIpICsgTWF0aC5wb3coaW0xLCAyKSksXG4gICAgZWlnZW4xOiB7XG4gICAgICByZTogcmUxLFxuICAgICAgaW06IGltMVxuICAgIH0sXG4gICAgZWlnZW4yOiB7XG4gICAgICByZTogcmUyLFxuICAgICAgaW06IGltMlxuICAgIH1cbiAgfTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBlaWdlbnZhbHVlczsiLCJcInVzZSBzdHJpY3RcIjtcblxuZnVuY3Rpb24gX3NsaWNlZFRvQXJyYXkoYXJyLCBpKSB7IHJldHVybiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB8fCBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB8fCBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkoYXJyLCBpKSB8fCBfbm9uSXRlcmFibGVSZXN0KCk7IH1cblxuZnVuY3Rpb24gX25vbkl0ZXJhYmxlUmVzdCgpIHsgdGhyb3cgbmV3IFR5cGVFcnJvcihcIkludmFsaWQgYXR0ZW1wdCB0byBkZXN0cnVjdHVyZSBub24taXRlcmFibGUgaW5zdGFuY2UuXFxuSW4gb3JkZXIgdG8gYmUgaXRlcmFibGUsIG5vbi1hcnJheSBvYmplY3RzIG11c3QgaGF2ZSBhIFtTeW1ib2wuaXRlcmF0b3JdKCkgbWV0aG9kLlwiKTsgfVxuXG5mdW5jdGlvbiBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkobywgbWluTGVuKSB7IGlmICghbykgcmV0dXJuOyBpZiAodHlwZW9mIG8gPT09IFwic3RyaW5nXCIpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB2YXIgbiA9IE9iamVjdC5wcm90b3R5cGUudG9TdHJpbmcuY2FsbChvKS5zbGljZSg4LCAtMSk7IGlmIChuID09PSBcIk9iamVjdFwiICYmIG8uY29uc3RydWN0b3IpIG4gPSBvLmNvbnN0cnVjdG9yLm5hbWU7IGlmIChuID09PSBcIk1hcFwiIHx8IG4gPT09IFwiU2V0XCIpIHJldHVybiBBcnJheS5mcm9tKG8pOyBpZiAobiA9PT0gXCJBcmd1bWVudHNcIiB8fCAvXig/OlVpfEkpbnQoPzo4fDE2fDMyKSg/OkNsYW1wZWQpP0FycmF5JC8udGVzdChuKSkgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IH1cblxuZnVuY3Rpb24gX2FycmF5TGlrZVRvQXJyYXkoYXJyLCBsZW4pIHsgaWYgKGxlbiA9PSBudWxsIHx8IGxlbiA+IGFyci5sZW5ndGgpIGxlbiA9IGFyci5sZW5ndGg7IGZvciAodmFyIGkgPSAwLCBhcnIyID0gbmV3IEFycmF5KGxlbik7IGkgPCBsZW47IGkrKykgeyBhcnIyW2ldID0gYXJyW2ldOyB9IHJldHVybiBhcnIyOyB9XG5cbmZ1bmN0aW9uIF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHsgaWYgKHR5cGVvZiBTeW1ib2wgPT09IFwidW5kZWZpbmVkXCIgfHwgIShTeW1ib2wuaXRlcmF0b3IgaW4gT2JqZWN0KGFycikpKSByZXR1cm47IHZhciBfYXJyID0gW107IHZhciBfbiA9IHRydWU7IHZhciBfZCA9IGZhbHNlOyB2YXIgX2UgPSB1bmRlZmluZWQ7IHRyeSB7IGZvciAodmFyIF9pID0gYXJyW1N5bWJvbC5pdGVyYXRvcl0oKSwgX3M7ICEoX24gPSAoX3MgPSBfaS5uZXh0KCkpLmRvbmUpOyBfbiA9IHRydWUpIHsgX2Fyci5wdXNoKF9zLnZhbHVlKTsgaWYgKGkgJiYgX2Fyci5sZW5ndGggPT09IGkpIGJyZWFrOyB9IH0gY2F0Y2ggKGVycikgeyBfZCA9IHRydWU7IF9lID0gZXJyOyB9IGZpbmFsbHkgeyB0cnkgeyBpZiAoIV9uICYmIF9pW1wicmV0dXJuXCJdICE9IG51bGwpIF9pW1wicmV0dXJuXCJdKCk7IH0gZmluYWxseSB7IGlmIChfZCkgdGhyb3cgX2U7IH0gfSByZXR1cm4gX2FycjsgfVxuXG5mdW5jdGlvbiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB7IGlmIChBcnJheS5pc0FycmF5KGFycikpIHJldHVybiBhcnI7IH1cblxudmFyIE1hdHJpeCA9IHJlcXVpcmUoJy4uLy4uJyk7XG5cbnZhciBfcmVxdWlyZSA9IHJlcXVpcmUoJy4uLy4uL0Vycm9yJyksXG4gICAgSU5WQUxJRF9QX05PUk0gPSBfcmVxdWlyZS5JTlZBTElEX1BfTk9STTtcbi8qKlxyXG4gKiBDYWxjdWxhdGVzIHRoZSBNYXRyaXggbm9ybSBvZiBhbnkgTWF0cml4IHdpdGggcmVzcGVjdCB0byB0aGUgY2hvaWNlIG9mIG5vcm0uPGJyPjxicj5cclxuICogXHJcbiAqIDEtbm9ybTogTWF4aW11bSBhYnNvbHV0ZSBjb2x1bW4gc3VtIG9mIHRoZSBNYXRyaXguPGJyPlxyXG4gKiAyLW5vcm06IFRoZSBsYXJnZXN0IHNpbmd1bGFyIHZhbHVlIG9mIE1hdHJpeC48YnI+XHJcbiAqIEluZmluaXR5LW5vcm06IE1heGltdW0gYWJzb2x1dGUgcm93IHN1bSBvZiB0aGUgTWF0cml4Ljxicj5cclxuICogRnJvYmVuaXVzLW5vcm06IEV1Y2xpZGVhbiBub3JtIGludmxvdmluZyBhbGwgZW50cmllcy48YnI+PGJyPlxyXG4gKiBcclxuICogVGhlIG5vcm1zIGFyZSBub3QgY2FjaGVkLlxyXG4gKiBAbWVtYmVyb2YgTWF0cml4XHJcbiAqIEBpbnN0YW5jZVxyXG4gKiBAcGFyYW0geygxfDJ8SW5maW5pdHl8J0YnKX0gcCAtIFRoZSBjaG9pY2Ugb2YgTWF0cml4IG5vcm1cclxuICogQHJldHVybnMge251bWJlcn0gVGhlIG5vcm0gb2YgdGhlIE1hdHJpeC5cclxuICovXG5cblxuZnVuY3Rpb24gbm9ybSgpIHtcbiAgdmFyIHAgPSBhcmd1bWVudHMubGVuZ3RoID4gMCAmJiBhcmd1bWVudHNbMF0gIT09IHVuZGVmaW5lZCA/IGFyZ3VtZW50c1swXSA6IDI7XG5cbiAgdmFyIF90aGlzJHNpemUgPSB0aGlzLnNpemUoKSxcbiAgICAgIF90aGlzJHNpemUyID0gX3NsaWNlZFRvQXJyYXkoX3RoaXMkc2l6ZSwgMiksXG4gICAgICByb3cgPSBfdGhpcyRzaXplMlswXSxcbiAgICAgIGNvbCA9IF90aGlzJHNpemUyWzFdO1xuXG4gIGlmIChwICE9PSAxICYmIHAgIT09IDIgJiYgcCAhPT0gSW5maW5pdHkgJiYgcCAhPT0gJ0YnKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfUF9OT1JNKTtcbiAgfVxuXG4gIHZhciBtYXRyaXggPSB0aGlzLl9tYXRyaXg7XG4gIHZhciByZXN1bHQgPSAwO1xuXG4gIGlmIChwID09PSAxKSB7XG4gICAgLy8gbWF4IG9mIGNvbHVtbiBzdW1cbiAgICBmb3IgKHZhciBqID0gMDsgaiA8IGNvbDsgaisrKSB7XG4gICAgICB2YXIgY29sdW1uU3VtID0gMDtcblxuICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCByb3c7IGkrKykge1xuICAgICAgICBjb2x1bW5TdW0gKz0gTWF0aC5hYnMobWF0cml4W2ldW2pdKTtcbiAgICAgIH1cblxuICAgICAgaWYgKGNvbHVtblN1bSA+IHJlc3VsdCkge1xuICAgICAgICByZXN1bHQgPSBjb2x1bW5TdW07XG4gICAgICB9XG4gICAgfVxuXG4gICAgcmV0dXJuIHJlc3VsdDtcbiAgfSAvLyBsYXJnZXN0IHNpbmd1bGFyIHZhbHVlXG5cblxuICBpZiAocCA9PT0gMikge1xuICAgIHZhciB0cmFuc3Bvc2UgPSBNYXRyaXgudHJhbnNwb3NlKHRoaXMpO1xuICAgIHZhciBNID0gTWF0cml4Lm11bHRpcGx5KHRyYW5zcG9zZSwgdGhpcyk7XG4gICAgdmFyIGVpZ2VudmFsdWVzID0gTS5laWdlbnZhbHVlcygpO1xuXG4gICAgZm9yICh2YXIgX2kyID0gMDsgX2kyIDwgZWlnZW52YWx1ZXMubGVuZ3RoOyBfaTIrKykge1xuICAgICAgdmFyIHZhbHVlID0gZWlnZW52YWx1ZXNbX2kyXS5nZXRNb2R1bHVzKCk7XG5cbiAgICAgIGlmICh2YWx1ZSA+IHJlc3VsdCkge1xuICAgICAgICByZXN1bHQgPSB2YWx1ZTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICByZXR1cm4gTWF0aC5zcXJ0KHJlc3VsdCk7XG4gIH1cblxuICBpZiAocCA9PT0gSW5maW5pdHkpIHtcbiAgICAvLyBtYXggb2Ygcm93IHN1bVxuICAgIGZvciAodmFyIF9pMyA9IDA7IF9pMyA8IHJvdzsgX2kzKyspIHtcbiAgICAgIHZhciByb3dTdW0gPSAwO1xuXG4gICAgICBmb3IgKHZhciBfaiA9IDA7IF9qIDwgY29sOyBfaisrKSB7XG4gICAgICAgIHJvd1N1bSArPSBNYXRoLmFicyhtYXRyaXhbX2kzXVtfal0pO1xuICAgICAgfVxuXG4gICAgICBpZiAocm93U3VtID4gcmVzdWx0KSB7XG4gICAgICAgIHJlc3VsdCA9IHJvd1N1bTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICByZXR1cm4gcmVzdWx0O1xuICB9IC8vIEZcblxuXG4gIGZvciAodmFyIF9pNCA9IDA7IF9pNCA8IHJvdzsgX2k0KyspIHtcbiAgICBmb3IgKHZhciBfajIgPSAwOyBfajIgPCBjb2w7IF9qMisrKSB7XG4gICAgICByZXN1bHQgKz0gTWF0aC5wb3cobWF0cml4W19pNF1bX2oyXSwgMik7XG4gICAgfVxuICB9XG5cbiAgcmV0dXJuIE1hdGguc3FydChyZXN1bHQpO1xufVxuXG47XG5tb2R1bGUuZXhwb3J0cyA9IG5vcm07IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbi8qKlxyXG4gKiBDYWxjdWxhdGVzIHRoZSBudWxsaXR5IG9mIGFueSBNYXRyaXgsIHdoaWNoIGlzIHRoZSBkaW1lbnNpb25cclxuICogb2YgdGhlIG51bGxzcGFjZS48YnI+PGJyPlxyXG4gKiBcclxuICogVGhlIG51bGxpdHkgaXMgY2FjaGVkLlxyXG4gKiBAbWVtYmVyb2YgTWF0cml4XHJcbiAqIEBpbnN0YW5jZVxyXG4gKiBAcmV0dXJucyB7bnVtYmVyfSBUaGUgbnVsbGl0eSBvZiB0aGUgbWF0cml4XHJcbiAqL1xuZnVuY3Rpb24gbnVsbGl0eSgpIHtcbiAgaWYgKHRoaXMuX251bGxpdHkgIT09IHVuZGVmaW5lZCkge1xuICAgIHJldHVybiB0aGlzLl9udWxsaXR5O1xuICB9XG5cbiAgdmFyIGNvbCA9IHRoaXMuc2l6ZSgpWzFdO1xuICB2YXIgcmFuayA9IHRoaXMucmFuaygpO1xuICB0aGlzLl9udWxsaXR5ID0gY29sIC0gcmFuaztcbiAgcmV0dXJuIHRoaXMuX251bGxpdHk7XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gbnVsbGl0eTsiLCJcInVzZSBzdHJpY3RcIjtcblxuZnVuY3Rpb24gX3NsaWNlZFRvQXJyYXkoYXJyLCBpKSB7IHJldHVybiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB8fCBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB8fCBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkoYXJyLCBpKSB8fCBfbm9uSXRlcmFibGVSZXN0KCk7IH1cblxuZnVuY3Rpb24gX25vbkl0ZXJhYmxlUmVzdCgpIHsgdGhyb3cgbmV3IFR5cGVFcnJvcihcIkludmFsaWQgYXR0ZW1wdCB0byBkZXN0cnVjdHVyZSBub24taXRlcmFibGUgaW5zdGFuY2UuXFxuSW4gb3JkZXIgdG8gYmUgaXRlcmFibGUsIG5vbi1hcnJheSBvYmplY3RzIG11c3QgaGF2ZSBhIFtTeW1ib2wuaXRlcmF0b3JdKCkgbWV0aG9kLlwiKTsgfVxuXG5mdW5jdGlvbiBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkobywgbWluTGVuKSB7IGlmICghbykgcmV0dXJuOyBpZiAodHlwZW9mIG8gPT09IFwic3RyaW5nXCIpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB2YXIgbiA9IE9iamVjdC5wcm90b3R5cGUudG9TdHJpbmcuY2FsbChvKS5zbGljZSg4LCAtMSk7IGlmIChuID09PSBcIk9iamVjdFwiICYmIG8uY29uc3RydWN0b3IpIG4gPSBvLmNvbnN0cnVjdG9yLm5hbWU7IGlmIChuID09PSBcIk1hcFwiIHx8IG4gPT09IFwiU2V0XCIpIHJldHVybiBBcnJheS5mcm9tKG8pOyBpZiAobiA9PT0gXCJBcmd1bWVudHNcIiB8fCAvXig/OlVpfEkpbnQoPzo4fDE2fDMyKSg/OkNsYW1wZWQpP0FycmF5JC8udGVzdChuKSkgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IH1cblxuZnVuY3Rpb24gX2FycmF5TGlrZVRvQXJyYXkoYXJyLCBsZW4pIHsgaWYgKGxlbiA9PSBudWxsIHx8IGxlbiA+IGFyci5sZW5ndGgpIGxlbiA9IGFyci5sZW5ndGg7IGZvciAodmFyIGkgPSAwLCBhcnIyID0gbmV3IEFycmF5KGxlbik7IGkgPCBsZW47IGkrKykgeyBhcnIyW2ldID0gYXJyW2ldOyB9IHJldHVybiBhcnIyOyB9XG5cbmZ1bmN0aW9uIF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHsgaWYgKHR5cGVvZiBTeW1ib2wgPT09IFwidW5kZWZpbmVkXCIgfHwgIShTeW1ib2wuaXRlcmF0b3IgaW4gT2JqZWN0KGFycikpKSByZXR1cm47IHZhciBfYXJyID0gW107IHZhciBfbiA9IHRydWU7IHZhciBfZCA9IGZhbHNlOyB2YXIgX2UgPSB1bmRlZmluZWQ7IHRyeSB7IGZvciAodmFyIF9pID0gYXJyW1N5bWJvbC5pdGVyYXRvcl0oKSwgX3M7ICEoX24gPSAoX3MgPSBfaS5uZXh0KCkpLmRvbmUpOyBfbiA9IHRydWUpIHsgX2Fyci5wdXNoKF9zLnZhbHVlKTsgaWYgKGkgJiYgX2Fyci5sZW5ndGggPT09IGkpIGJyZWFrOyB9IH0gY2F0Y2ggKGVycikgeyBfZCA9IHRydWU7IF9lID0gZXJyOyB9IGZpbmFsbHkgeyB0cnkgeyBpZiAoIV9uICYmIF9pW1wicmV0dXJuXCJdICE9IG51bGwpIF9pW1wicmV0dXJuXCJdKCk7IH0gZmluYWxseSB7IGlmIChfZCkgdGhyb3cgX2U7IH0gfSByZXR1cm4gX2FycjsgfVxuXG5mdW5jdGlvbiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB7IGlmIChBcnJheS5pc0FycmF5KGFycikpIHJldHVybiBhcnI7IH1cblxudmFyIE1hdHJpeCA9IHJlcXVpcmUoJy4uLy4uJyk7XG4vKipcclxuICogQ2FsY3VsYXRlcyB0aGUgcmFuayBvZiBhbnkgTWF0cml4LFxyXG4gKiB3aGljaCBpcyB0aGUgZGltZW5zaW9uIG9mIHRoZSByb3cgc3BhY2UuPGJyPjxicj5cclxuICogXHJcbiAqIFRoZSByYW5rIGlzIGNhY2hlZC5cclxuICogQG1lbWJlcm9mIE1hdHJpeFxyXG4gKiBAaW5zdGFuY2VcclxuICogQHJldHVybnMge251bWJlcn0gVGhlIHJhbmsgb2YgdGhlIE1hdHJpeFxyXG4gKi9cblxuXG5mdW5jdGlvbiByYW5rKCkge1xuICBpZiAodGhpcy5fcmFuayAhPT0gdW5kZWZpbmVkKSB7XG4gICAgcmV0dXJuIHRoaXMuX3Jhbms7XG4gIH1cblxuICB2YXIgRVBTSUxPTiA9IDEgLyAoTWF0aC5wb3coMTAsIHRoaXMuX2RpZ2l0KSAqIDIpO1xuICB2YXIgUiA9IE1hdHJpeC5RUih0aGlzKVsxXTtcbiAgdmFyIG1hdHJpeFIgPSBSLl9tYXRyaXg7XG5cbiAgdmFyIF9SJHNpemUgPSBSLnNpemUoKSxcbiAgICAgIF9SJHNpemUyID0gX3NsaWNlZFRvQXJyYXkoX1Ikc2l6ZSwgMiksXG4gICAgICByb3cgPSBfUiRzaXplMlswXSxcbiAgICAgIGNvbCA9IF9SJHNpemUyWzFdO1xuXG4gIGlmIChyb3cgPT09IDApIHtcbiAgICB0aGlzLl9yYW5rID0gMTtcbiAgICByZXR1cm4gMTtcbiAgfVxuXG4gIHZhciByayA9IDA7XG5cbiAgZm9yICh2YXIgaSA9IDA7IGkgPCByb3c7IGkrKykge1xuICAgIGZvciAodmFyIGogPSBpOyBqIDwgY29sOyBqKyspIHtcbiAgICAgIGlmIChNYXRoLmFicyhtYXRyaXhSW2ldW2pdKSA+PSBFUFNJTE9OKSB7XG4gICAgICAgIHJrKys7XG4gICAgICAgIGJyZWFrO1xuICAgICAgfVxuICAgIH1cbiAgfVxuXG4gIHRoaXMuX3JhbmsgPSByaztcbiAgcmV0dXJuIHJrO1xufVxuXG47XG5tb2R1bGUuZXhwb3J0cyA9IHJhbms7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbi8qKlxyXG4gKiBDYWxjdWxhdGVzIHRoZSBzaXplIG9mIGFueSBNYXRyaXgsXHJcbiAqIHdoaWNoIGlzIGluIHRoZSBmb3JtIG9mIFtyb3csIGNvbHVtbl0uPGJyPjxicj5cclxuICogXHJcbiAqIFRoZSBzaXplIG9mIE1hdHJpeCBpcyBjYWNoZWQuXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQGluc3RhbmNlXHJcbiAqIEByZXR1cm5zIHtudW1iZXJbXX0gVGhlIG51bWJlciBvZiByb3dzIGFuZCBjb2x1bW5zIG9mIGEgTWF0cml4XHJcbiAqL1xuZnVuY3Rpb24gc2l6ZSgpIHtcbiAgaWYgKHRoaXMuX3NpemUgIT09IHVuZGVmaW5lZCkge1xuICAgIHJldHVybiB0aGlzLl9zaXplO1xuICB9XG5cbiAgdmFyIEEgPSB0aGlzLl9tYXRyaXg7XG5cbiAgaWYgKEEubGVuZ3RoID09PSAwKSB7XG4gICAgdGhpcy5fc2l6ZSA9IFswLCAwXTtcbiAgICByZXR1cm4gdGhpcy5fc2l6ZTtcbiAgfVxuXG4gIHRoaXMuX3NpemUgPSBbQS5sZW5ndGgsIEFbMF0ubGVuZ3RoXTtcbiAgcmV0dXJuIHRoaXMuX3NpemU7XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gc2l6ZTsiLCJcInVzZSBzdHJpY3RcIjtcblxudmFyIF9yZXF1aXJlID0gcmVxdWlyZSgnLi4vLi4vRXJyb3InKSxcbiAgICBJTlZBTElEX1NRVUFSRV9NQVRSSVggPSBfcmVxdWlyZS5JTlZBTElEX1NRVUFSRV9NQVRSSVg7XG4vKipcclxuICogQ2FsY3VsYXRlcyB0aGUgdHJhY2Ugb2YgYW55IHNxdWFyZSBNYXRyaXgsXHJcbiAqIHdoaWNoIGlzIHRoZSBzdW0gb2YgYWxsIGVudHJpZXMgb24gdGhlIG1haW4gZGlhZ29uYWwuPGJyPjxicj5cclxuICogXHJcbiAqIFRoZSB0cmFjZSBpcyBjYWNoZWQuXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQGluc3RhbmNlXHJcbiAqIEByZXR1cm5zIHtudW1iZXJ9IFRoZSB0cmFjZSBvZiB0aGUgc3F1YXJlIE1hdHJpeC5cclxuICovXG5cblxuZnVuY3Rpb24gdHJhY2UoKSB7XG4gIHZhciBpc1NxdWFyZSA9IHRoaXMuX2lzU3F1YXJlICE9PSB1bmRlZmluZWQgPyB0aGlzLl9pc1NxdWFyZSA6IHRoaXMuaXNTcXVhcmUoKTtcblxuICBpZiAoIWlzU3F1YXJlKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfU1FVQVJFX01BVFJJWCk7XG4gIH1cblxuICBpZiAodGhpcy5fdHJhY2UgIT09IHVuZGVmaW5lZCkge1xuICAgIHJldHVybiB0aGlzLl90cmFjZTtcbiAgfVxuXG4gIHZhciBBID0gdGhpcy5fbWF0cml4O1xuICB2YXIgc2l6ZSA9IEEubGVuZ3RoO1xuICB2YXIgdHIgPSAwO1xuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgc2l6ZTsgaSsrKSB7XG4gICAgdHIgKz0gQVtpXVtpXTtcbiAgfVxuXG4gIHRoaXMuX3RyYWNlID0gdHI7XG4gIHJldHVybiB0cjtcbn1cblxuO1xubW9kdWxlLmV4cG9ydHMgPSB0cmFjZTsiLCJcInVzZSBzdHJpY3RcIjtcblxuZnVuY3Rpb24gX3NsaWNlZFRvQXJyYXkoYXJyLCBpKSB7IHJldHVybiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB8fCBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB8fCBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkoYXJyLCBpKSB8fCBfbm9uSXRlcmFibGVSZXN0KCk7IH1cblxuZnVuY3Rpb24gX25vbkl0ZXJhYmxlUmVzdCgpIHsgdGhyb3cgbmV3IFR5cGVFcnJvcihcIkludmFsaWQgYXR0ZW1wdCB0byBkZXN0cnVjdHVyZSBub24taXRlcmFibGUgaW5zdGFuY2UuXFxuSW4gb3JkZXIgdG8gYmUgaXRlcmFibGUsIG5vbi1hcnJheSBvYmplY3RzIG11c3QgaGF2ZSBhIFtTeW1ib2wuaXRlcmF0b3JdKCkgbWV0aG9kLlwiKTsgfVxuXG5mdW5jdGlvbiBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkobywgbWluTGVuKSB7IGlmICghbykgcmV0dXJuOyBpZiAodHlwZW9mIG8gPT09IFwic3RyaW5nXCIpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB2YXIgbiA9IE9iamVjdC5wcm90b3R5cGUudG9TdHJpbmcuY2FsbChvKS5zbGljZSg4LCAtMSk7IGlmIChuID09PSBcIk9iamVjdFwiICYmIG8uY29uc3RydWN0b3IpIG4gPSBvLmNvbnN0cnVjdG9yLm5hbWU7IGlmIChuID09PSBcIk1hcFwiIHx8IG4gPT09IFwiU2V0XCIpIHJldHVybiBBcnJheS5mcm9tKG8pOyBpZiAobiA9PT0gXCJBcmd1bWVudHNcIiB8fCAvXig/OlVpfEkpbnQoPzo4fDE2fDMyKSg/OkNsYW1wZWQpP0FycmF5JC8udGVzdChuKSkgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IH1cblxuZnVuY3Rpb24gX2FycmF5TGlrZVRvQXJyYXkoYXJyLCBsZW4pIHsgaWYgKGxlbiA9PSBudWxsIHx8IGxlbiA+IGFyci5sZW5ndGgpIGxlbiA9IGFyci5sZW5ndGg7IGZvciAodmFyIGkgPSAwLCBhcnIyID0gbmV3IEFycmF5KGxlbik7IGkgPCBsZW47IGkrKykgeyBhcnIyW2ldID0gYXJyW2ldOyB9IHJldHVybiBhcnIyOyB9XG5cbmZ1bmN0aW9uIF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHsgaWYgKHR5cGVvZiBTeW1ib2wgPT09IFwidW5kZWZpbmVkXCIgfHwgIShTeW1ib2wuaXRlcmF0b3IgaW4gT2JqZWN0KGFycikpKSByZXR1cm47IHZhciBfYXJyID0gW107IHZhciBfbiA9IHRydWU7IHZhciBfZCA9IGZhbHNlOyB2YXIgX2UgPSB1bmRlZmluZWQ7IHRyeSB7IGZvciAodmFyIF9pID0gYXJyW1N5bWJvbC5pdGVyYXRvcl0oKSwgX3M7ICEoX24gPSAoX3MgPSBfaS5uZXh0KCkpLmRvbmUpOyBfbiA9IHRydWUpIHsgX2Fyci5wdXNoKF9zLnZhbHVlKTsgaWYgKGkgJiYgX2Fyci5sZW5ndGggPT09IGkpIGJyZWFrOyB9IH0gY2F0Y2ggKGVycikgeyBfZCA9IHRydWU7IF9lID0gZXJyOyB9IGZpbmFsbHkgeyB0cnkgeyBpZiAoIV9uICYmIF9pW1wicmV0dXJuXCJdICE9IG51bGwpIF9pW1wicmV0dXJuXCJdKCk7IH0gZmluYWxseSB7IGlmIChfZCkgdGhyb3cgX2U7IH0gfSByZXR1cm4gX2FycjsgfVxuXG5mdW5jdGlvbiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB7IGlmIChBcnJheS5pc0FycmF5KGFycikpIHJldHVybiBhcnI7IH1cblxuLyoqXHJcbiAqIERldGVybWluZXMgd2hldGhlciBhIE1hdHJpeCBpcyBkaWFnb25hbCBvciBub3QuPGJyPjxicj5cclxuICogXHJcbiAqIERpYWdvbmFsIE1hdHJpeCBpcyBhIE1hdHJpeCBpbiB3aGljaCB0aGUgZW50cmllcyBvdXRzaWRlIHRoZSBtYWluIGRpYWdvbmFsXHJcbiAqIGFyZSBhbGwgemVyby4gTm90ZSB0aGF0IHRoZSB0ZXJtIGRpYWdvbmFsIHJlZmVycyB0byByZWN0YW5ndWxhciBkaWFnb25hbC48YnI+PGJyPlxyXG4gKiBcclxuICogVGhlIHJlc3VsdCBpcyBjYWNoZWQuXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQGluc3RhbmNlXHJcbiAqIEBwYXJhbSB7bnVtYmVyfSBbZGlnaXQ9OF0gLSBOdW1iZXIgb2Ygc2lnbmlmaWNhbnQgZGlnaXRzXHJcbiAqIEByZXR1cm5zIHtib29sZWFufSBSZXR1cm5zIHJ1ZSBpZiB0aGUgTWF0cml4IGlzIGRpYWdvbmFsIE1hdHJpeFxyXG4gKi9cbmZ1bmN0aW9uIGlzRGlhZ29uYWwoKSB7XG4gIHZhciBkaWdpdCA9IGFyZ3VtZW50cy5sZW5ndGggPiAwICYmIGFyZ3VtZW50c1swXSAhPT0gdW5kZWZpbmVkID8gYXJndW1lbnRzWzBdIDogdGhpcy5fZGlnaXQ7XG5cbiAgaWYgKHRoaXMuX2lzRGlhZ29uYWwgIT09IHVuZGVmaW5lZCkge1xuICAgIHJldHVybiB0aGlzLl9pc0RpYWdvbmFsO1xuICB9XG5cbiAgdmFyIEVQU0lMT04gPSAxIC8gKE1hdGgucG93KDEwLCBkaWdpdCkgKiAyKTtcbiAgdmFyIEEgPSB0aGlzLl9tYXRyaXg7XG5cbiAgdmFyIF90aGlzJHNpemUgPSB0aGlzLnNpemUoKSxcbiAgICAgIF90aGlzJHNpemUyID0gX3NsaWNlZFRvQXJyYXkoX3RoaXMkc2l6ZSwgMiksXG4gICAgICByb3cgPSBfdGhpcyRzaXplMlswXSxcbiAgICAgIGNvbCA9IF90aGlzJHNpemUyWzFdO1xuXG4gIGlmIChyb3cgPT09IDApIHtcbiAgICB0aGlzLl9pc0RpYWdvbmFsID0gdHJ1ZTtcbiAgICByZXR1cm4gdHJ1ZTtcbiAgfVxuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgcm93OyBpKyspIHtcbiAgICBmb3IgKHZhciBqID0gMDsgaiA8IGNvbDsgaisrKSB7XG4gICAgICBpZiAoaSAhPT0gaiAmJiBNYXRoLmFicyhBW2ldW2pdKSA+PSBFUFNJTE9OKSB7XG4gICAgICAgIHRoaXMuaXNEaWFnb25hbCA9IGZhbHNlO1xuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgICB9XG4gICAgfVxuICB9XG5cbiAgdGhpcy5faXNEaWFnb25hbCA9IHRydWU7XG4gIHJldHVybiB0cnVlO1xufVxuXG47XG5tb2R1bGUuZXhwb3J0cyA9IGlzRGlhZ29uYWw7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbmZ1bmN0aW9uIF9zbGljZWRUb0FycmF5KGFyciwgaSkgeyByZXR1cm4gX2FycmF5V2l0aEhvbGVzKGFycikgfHwgX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgfHwgX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KGFyciwgaSkgfHwgX25vbkl0ZXJhYmxlUmVzdCgpOyB9XG5cbmZ1bmN0aW9uIF9ub25JdGVyYWJsZVJlc3QoKSB7IHRocm93IG5ldyBUeXBlRXJyb3IoXCJJbnZhbGlkIGF0dGVtcHQgdG8gZGVzdHJ1Y3R1cmUgbm9uLWl0ZXJhYmxlIGluc3RhbmNlLlxcbkluIG9yZGVyIHRvIGJlIGl0ZXJhYmxlLCBub24tYXJyYXkgb2JqZWN0cyBtdXN0IGhhdmUgYSBbU3ltYm9sLml0ZXJhdG9yXSgpIG1ldGhvZC5cIik7IH1cblxuZnVuY3Rpb24gX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KG8sIG1pbkxlbikgeyBpZiAoIW8pIHJldHVybjsgaWYgKHR5cGVvZiBvID09PSBcInN0cmluZ1wiKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgdmFyIG4gPSBPYmplY3QucHJvdG90eXBlLnRvU3RyaW5nLmNhbGwobykuc2xpY2UoOCwgLTEpOyBpZiAobiA9PT0gXCJPYmplY3RcIiAmJiBvLmNvbnN0cnVjdG9yKSBuID0gby5jb25zdHJ1Y3Rvci5uYW1lOyBpZiAobiA9PT0gXCJNYXBcIiB8fCBuID09PSBcIlNldFwiKSByZXR1cm4gQXJyYXkuZnJvbShvKTsgaWYgKG4gPT09IFwiQXJndW1lbnRzXCIgfHwgL14oPzpVaXxJKW50KD86OHwxNnwzMikoPzpDbGFtcGVkKT9BcnJheSQvLnRlc3QobikpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB9XG5cbmZ1bmN0aW9uIF9hcnJheUxpa2VUb0FycmF5KGFyciwgbGVuKSB7IGlmIChsZW4gPT0gbnVsbCB8fCBsZW4gPiBhcnIubGVuZ3RoKSBsZW4gPSBhcnIubGVuZ3RoOyBmb3IgKHZhciBpID0gMCwgYXJyMiA9IG5ldyBBcnJheShsZW4pOyBpIDwgbGVuOyBpKyspIHsgYXJyMltpXSA9IGFycltpXTsgfSByZXR1cm4gYXJyMjsgfVxuXG5mdW5jdGlvbiBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB7IGlmICh0eXBlb2YgU3ltYm9sID09PSBcInVuZGVmaW5lZFwiIHx8ICEoU3ltYm9sLml0ZXJhdG9yIGluIE9iamVjdChhcnIpKSkgcmV0dXJuOyB2YXIgX2FyciA9IFtdOyB2YXIgX24gPSB0cnVlOyB2YXIgX2QgPSBmYWxzZTsgdmFyIF9lID0gdW5kZWZpbmVkOyB0cnkgeyBmb3IgKHZhciBfaSA9IGFycltTeW1ib2wuaXRlcmF0b3JdKCksIF9zOyAhKF9uID0gKF9zID0gX2kubmV4dCgpKS5kb25lKTsgX24gPSB0cnVlKSB7IF9hcnIucHVzaChfcy52YWx1ZSk7IGlmIChpICYmIF9hcnIubGVuZ3RoID09PSBpKSBicmVhazsgfSB9IGNhdGNoIChlcnIpIHsgX2QgPSB0cnVlOyBfZSA9IGVycjsgfSBmaW5hbGx5IHsgdHJ5IHsgaWYgKCFfbiAmJiBfaVtcInJldHVyblwiXSAhPSBudWxsKSBfaVtcInJldHVyblwiXSgpOyB9IGZpbmFsbHkgeyBpZiAoX2QpIHRocm93IF9lOyB9IH0gcmV0dXJuIF9hcnI7IH1cblxuZnVuY3Rpb24gX2FycmF5V2l0aEhvbGVzKGFycikgeyBpZiAoQXJyYXkuaXNBcnJheShhcnIpKSByZXR1cm4gYXJyOyB9XG5cbi8qKlxyXG4gKiBEZXRlcm1pbmVzIHdoZXRoZXIgYSBNYXRyaXggaXMgbG93ZXIgdHJpYW5ndWxhciBNYXRyaXggb3Igbm90Ljxicj48YnI+XHJcbiAqIFxyXG4gKiBMb3dlciB0cmlhbmd1bGFyIE1hdHJpeCBpcyBhIE1hdHJpeCBpbiB3aGljaCBhbGwgdGhlIGVudHJpZXNcclxuICogYWJvdmUgdGhlIG1haW4gZGlhZ29uYWwgYXJlIHplcm8uIE5vdGUgdGhhdCBpdCBjYW4gYmUgYXBwbGllZFxyXG4gKiB0byBhbnkgbm9uLXNxdWFyZSBNYXRyaXguPGJyPjxicj5cclxuICogXHJcbiAqIFRoZSByZXN1bHQgaXMgY2FjaGVkLlxyXG4gKiBAbWVtYmVyb2YgTWF0cml4XHJcbiAqIEBpbnN0YW5jZVxyXG4gKiBAcGFyYW0ge251bWJlcn0gW2RpZ2l0PThdIC0gTnVtYmVyIG9mIHNpZ25pZmljYW50IGRpZ2l0c1xyXG4gKiBAcmV0dXJucyB7Ym9vbGVhbn0gUmV0dXJucyB0cnVlIGlmIHRoZSBNYXRyaXggaXMgbG93ZXIgdHJpYW5ndWxhclxyXG4gKi9cbmZ1bmN0aW9uIGlzTG93ZXJUcmlhbmd1bGFyKCkge1xuICB2YXIgZGlnaXQgPSBhcmd1bWVudHMubGVuZ3RoID4gMCAmJiBhcmd1bWVudHNbMF0gIT09IHVuZGVmaW5lZCA/IGFyZ3VtZW50c1swXSA6IHRoaXMuX2RpZ2l0O1xuXG4gIGlmICh0aGlzLl9pc0xvd2VyVHJpYW5ndWxhciAhPT0gdW5kZWZpbmVkKSB7XG4gICAgcmV0dXJuIHRoaXMuX2lzTG93ZXJUcmlhbmd1bGFyO1xuICB9XG5cbiAgdmFyIEVQU0lMT04gPSAxIC8gKE1hdGgucG93KDEwLCBkaWdpdCkgKiAyKTtcbiAgdmFyIEEgPSB0aGlzLl9tYXRyaXg7XG5cbiAgdmFyIF90aGlzJHNpemUgPSB0aGlzLnNpemUoKSxcbiAgICAgIF90aGlzJHNpemUyID0gX3NsaWNlZFRvQXJyYXkoX3RoaXMkc2l6ZSwgMiksXG4gICAgICByb3cgPSBfdGhpcyRzaXplMlswXSxcbiAgICAgIGNvbCA9IF90aGlzJHNpemUyWzFdO1xuXG4gIGlmIChyb3cgPT09IDApIHtcbiAgICAvLyBbXVxuICAgIHRoaXMuX2lzTG93ZXJUcmlhbmd1bGFyID0gdHJ1ZTtcbiAgICByZXR1cm4gdHJ1ZTtcbiAgfVxuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgcm93OyBpKyspIHtcbiAgICBmb3IgKHZhciBqID0gaSArIDE7IGogPCBjb2w7IGorKykge1xuICAgICAgaWYgKE1hdGguYWJzKEFbaV1bal0pID49IEVQU0lMT04pIHtcbiAgICAgICAgdGhpcy5faXNMb3dlclRyaWFuZ3VsYXIgPSBmYWxzZTtcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgfVxuICAgIH1cbiAgfVxuXG4gIHRoaXMuX2lzTG93ZXJUcmlhbmd1bGFyID0gdHJ1ZTtcbiAgcmV0dXJuIHRydWU7XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gaXNMb3dlclRyaWFuZ3VsYXI7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbi8qKlxyXG4gKiBEZXRlcm1pbmVzIHdoZXRoZXIgYSBzcXVhcmUgTWF0cml4IGlzIG9ydGhvZ29uYWwgb3Igbm90Ljxicj48YnI+XHJcbiAqIFxyXG4gKiBPcnRob2dvbmFsIE1hdHJpeCBpcyBhIE1hdHJpeCBpbiB3aGljaCBhbGwgcm93cyBhbmQgY29sdW1ucyBhcmVcclxuICogb3J0aG9ub3JtYWwgdmVjdG9ycy48YnI+PGJyPlxyXG4gKiBcclxuICogVGhlIHJlc3VsdCBpcyBjYWNoZWQuXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQGluc3RhbmNlXHJcbiAqIEBwYXJhbSB7bnVtYmVyfSBbZGlnaXQ9OF0gLSBOdW1iZXIgb2Ygc2lnbmlmaWNhbnQgZGlnaXRzXHJcbiAqIEByZXR1cm5zIHtib29sZWFufSBSZXR1cm5zIHRydWUgaWYgdGhlIHNxdWFyZSBNYXRyaXggaXMgb3J0aG9nb25hbFxyXG4gKi9cbmZ1bmN0aW9uIGlzT3J0aG9nb25hbCgpIHtcbiAgdmFyIGRpZ2l0ID0gYXJndW1lbnRzLmxlbmd0aCA+IDAgJiYgYXJndW1lbnRzWzBdICE9PSB1bmRlZmluZWQgPyBhcmd1bWVudHNbMF0gOiB0aGlzLl9kaWdpdDtcblxuICBpZiAodGhpcy5faXNPcnRob2dvbmFsICE9PSB1bmRlZmluZWQpIHtcbiAgICByZXR1cm4gdGhpcy5faXNPcnRob2dvbmFsO1xuICB9XG5cbiAgaWYgKCF0aGlzLmlzU3F1YXJlKCkpIHtcbiAgICB0aGlzLl9pc09ydGhvZ29uYWwgPSBmYWxzZTtcbiAgICByZXR1cm4gZmFsc2U7XG4gIH1cblxuICB2YXIgQSA9IHRoaXMuX21hdHJpeDtcbiAgdmFyIEVQU0lMT04gPSAxIC8gKE1hdGgucG93KDEwLCBkaWdpdCkgKiAyKTtcbiAgdmFyIHNpemUgPSBBLmxlbmd0aDtcblxuICBmb3IgKHZhciBpID0gMDsgaSA8IHNpemU7IGkrKykge1xuICAgIGZvciAodmFyIGogPSBpOyBqIDwgc2l6ZTsgaisrKSB7XG4gICAgICB2YXIgZW50cnkgPSAwO1xuXG4gICAgICBmb3IgKHZhciBrID0gMDsgayA8IHNpemU7IGsrKykge1xuICAgICAgICBlbnRyeSArPSBBW2ldW2tdICogQVtqXVtrXTtcbiAgICAgIH1cblxuICAgICAgaWYgKGkgPT09IGogJiYgTWF0aC5hYnMoZW50cnkgLSAxKSA+PSBFUFNJTE9OKSB7XG4gICAgICAgIHRoaXMuX2lzT3J0aG9nb25hbCA9IGZhbHNlO1xuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgICB9XG5cbiAgICAgIGlmIChpICE9PSBqICYmIE1hdGguYWJzKGVudHJ5KSA+PSBFUFNJTE9OKSB7XG4gICAgICAgIHRoaXMuX2lzT3J0aG9nb25hbCA9IGZhbHNlO1xuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgICB9XG4gICAgfVxuICB9XG5cbiAgdGhpcy5faXNPcnRob2dvbmFsID0gdHJ1ZTtcbiAgcmV0dXJuIHRydWU7XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gaXNPcnRob2dvbmFsOyIsIlwidXNlIHN0cmljdFwiO1xuXG4vKipcclxuICogRGV0ZXJtaW5lcyB3aGV0aGVyIGEgc3F1YXJlIE1hdHJpeCBpcyBza2V3IHN5bW1ldHJpYyBvciBub3QuPGJyPjxicj5cclxuICogXHJcbiAqIFNrZXcgc3ltbWV0cmljIE1hdHJpeCBpcyBhIHNxdWFyZSBNYXRyaXggd2hvc2UgdHJhbnNwb3NlIGVxdWFscyBpdHMgbmVnYXRpdmUuPGJyPjxicj5cclxuICogXHJcbiAqIFRoZSByZXN1bHQgaXMgY2FjaGVkLlxyXG4gKiBAbWVtYmVyb2YgTWF0cml4XHJcbiAqIEBpbnN0YW5jZVxyXG4gKiBAcGFyYW0ge251bWJlcn0gW2RpZ2l0PThdIC0gTnVtYmVyIG9mIHNpZ25pZmljYW50IGRpZ2l0c1xyXG4gKiBAcmV0dXJucyB7Ym9vbGVhbn0gUmV0dXJucyB0cnVlIGlmIHRoZSBzcXVhcmUgTWF0cml4IGlzIHNrZXcgc3ltbWV0cmljXHJcbiAqL1xuZnVuY3Rpb24gaXNTa2V3U3ltbWV0cmljKCkge1xuICB2YXIgZGlnaXQgPSBhcmd1bWVudHMubGVuZ3RoID4gMCAmJiBhcmd1bWVudHNbMF0gIT09IHVuZGVmaW5lZCA/IGFyZ3VtZW50c1swXSA6IHRoaXMuX2RpZ2l0O1xuXG4gIGlmICh0aGlzLl9pc1NrZXdTeW1tZXRyaWMgIT09IHVuZGVmaW5lZCkge1xuICAgIHJldHVybiB0aGlzLl9pc1NrZXdTeW1tZXRyaWM7XG4gIH1cblxuICBpZiAoIXRoaXMuaXNTcXVhcmUoKSkge1xuICAgIHRoaXMuX2lzU2tld1N5bW1ldHJpYyA9IGZhbHNlO1xuICAgIHJldHVybiBmYWxzZTtcbiAgfVxuXG4gIHZhciBBID0gdGhpcy5fbWF0cml4O1xuICB2YXIgRVBTSUxPTiA9IDEgLyAoTWF0aC5wb3coMTAsIGRpZ2l0KSAqIDIpO1xuICB2YXIgc2l6ZSA9IEEubGVuZ3RoO1xuXG4gIGlmIChzaXplID09PSAwKSB7XG4gICAgdGhpcy5faXNTa2V3U3ltbWV0cmljID0gdHJ1ZTtcbiAgICByZXR1cm4gdHJ1ZTsgLy8gW11cbiAgfVxuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgc2l6ZTsgaSsrKSB7XG4gICAgZm9yICh2YXIgaiA9IDA7IGogPCBpOyBqKyspIHtcbiAgICAgIGlmIChNYXRoLmFicyhBW2ldW2pdICsgQVtqXVtpXSkgPj0gRVBTSUxPTikge1xuICAgICAgICB0aGlzLl9pc1NrZXdTeW1tZXRyaWMgPSBmYWxzZTtcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgfVxuICAgIH1cbiAgfVxuXG4gIHRoaXMuX2lzU2tld1N5bW1ldHJpYyA9IHRydWU7XG4gIHJldHVybiB0cnVlO1xufVxuXG47XG5tb2R1bGUuZXhwb3J0cyA9IGlzU2tld1N5bW1ldHJpYzsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIERldGVybWluZXMgd2hldGhlciBhIE1hdHJpeCBpcyBzcXVhcmUgb3Igbm90Ljxicj48YnI+XHJcbiAqIFxyXG4gKiBTcXVhcmUgTWF0cml4IGlzIGEgTWF0cml4IHdpdGggc2FtZSBudW1iZXIgb2Ygcm93cyBhbmQgY29sdW1ucy48YnI+PGJyPlxyXG4gKiBcclxuICogVGhlIHJlc3VsdCBpcyBjYWNoZWQuXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQGluc3RhbmNlXHJcbiAqIEByZXR1cm5zIHtib29sZWFufSBSZXR1cm5zIHRydWUgaWYgdGhlIE1hdHJpeCBpcyBzcXVhcmVcclxuICovXG5mdW5jdGlvbiBpc1NxdWFyZSgpIHtcbiAgaWYgKHRoaXMuX2lzU3F1YXJlICE9PSB1bmRlZmluZWQpIHtcbiAgICByZXR1cm4gdGhpcy5faXNTcXVhcmU7XG4gIH1cblxuICB2YXIgQSA9IHRoaXMuX21hdHJpeDtcblxuICBpZiAoQS5sZW5ndGggPT09IDApIHtcbiAgICAvLyAweDAgbWF0cml4XG4gICAgdGhpcy5faXNTcXVhcmUgPSB0cnVlO1xuICAgIHJldHVybiB0cnVlO1xuICB9XG5cbiAgdGhpcy5faXNTcXVhcmUgPSBBLmxlbmd0aCA9PT0gQVswXS5sZW5ndGg7XG4gIHJldHVybiB0aGlzLl9pc1NxdWFyZTtcbn1cblxuO1xubW9kdWxlLmV4cG9ydHMgPSBpc1NxdWFyZTsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIERldGVybWluZXMgd2hldGhlciBhIHNxdWFyZSBNYXRyaXggaXMgc3ltbWV0cmljIG9yIG5vdC48YnI+PGJyPlxyXG4gKiBcclxuICogU3ltbWV0cmljIE1hdHJpeCBpcyBhIHNxdWFyZSBNYXRyaXggdGhhdCBpcyBlcXVhbCB0byBpdHMgdHJhbnNwb3NlLjxicj48YnI+XHJcbiAqIFxyXG4gKiBUaGUgcmVzdWx0IGlzIGNhY2hlZC5cclxuICogQG1lbWJlcm9mIE1hdHJpeFxyXG4gKiBAaW5zdGFuY2VcclxuICogQHBhcmFtIHtudW1iZXJ9IFtkaWdpdD04XSAtIE51bWJlciBvZiBzaWduaWZpY2FudCBkaWdpdHNcclxuICogQHJldHVybnMge2Jvb2xlYW59IFJldHVybnMgdHJ1ZSBpZiB0aGUgc3F1YXJlIE1hdHJpeCBpcyBzeW1tZXRyaWNcclxuICovXG5mdW5jdGlvbiBpc1N5bW1ldHJpYygpIHtcbiAgdmFyIGRpZ2l0ID0gYXJndW1lbnRzLmxlbmd0aCA+IDAgJiYgYXJndW1lbnRzWzBdICE9PSB1bmRlZmluZWQgPyBhcmd1bWVudHNbMF0gOiB0aGlzLl9kaWdpdDtcblxuICBpZiAodGhpcy5faXNTeW1tZXRyaWMgIT09IHVuZGVmaW5lZCkge1xuICAgIHJldHVybiB0aGlzLl9pc1N5bW1ldHJpYztcbiAgfVxuXG4gIGlmICghdGhpcy5pc1NxdWFyZSgpKSB7XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG5cbiAgdmFyIEEgPSB0aGlzLl9tYXRyaXg7XG4gIHZhciBFUFNJTE9OID0gMSAvIChNYXRoLnBvdygxMCwgZGlnaXQpICogMik7XG4gIHZhciBzaXplID0gQS5sZW5ndGg7XG5cbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBzaXplOyBpKyspIHtcbiAgICBmb3IgKHZhciBqID0gMDsgaiA8PSBpOyBqKyspIHtcbiAgICAgIGlmIChNYXRoLmFicyhBW2ldW2pdIC0gQVtqXVtpXSkgPj0gRVBTSUxPTikge1xuICAgICAgICB0aGlzLl9pc1N5bW1ldHJpYyA9IGZhbHNlO1xuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgICB9XG4gICAgfVxuICB9XG5cbiAgdGhpcy5faXNTeW1tZXRyaWMgPSB0cnVlO1xuICByZXR1cm4gdHJ1ZTtcbn1cblxuO1xubW9kdWxlLmV4cG9ydHMgPSBpc1N5bW1ldHJpYzsiLCJcInVzZSBzdHJpY3RcIjtcblxuZnVuY3Rpb24gX3NsaWNlZFRvQXJyYXkoYXJyLCBpKSB7IHJldHVybiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB8fCBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB8fCBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkoYXJyLCBpKSB8fCBfbm9uSXRlcmFibGVSZXN0KCk7IH1cblxuZnVuY3Rpb24gX25vbkl0ZXJhYmxlUmVzdCgpIHsgdGhyb3cgbmV3IFR5cGVFcnJvcihcIkludmFsaWQgYXR0ZW1wdCB0byBkZXN0cnVjdHVyZSBub24taXRlcmFibGUgaW5zdGFuY2UuXFxuSW4gb3JkZXIgdG8gYmUgaXRlcmFibGUsIG5vbi1hcnJheSBvYmplY3RzIG11c3QgaGF2ZSBhIFtTeW1ib2wuaXRlcmF0b3JdKCkgbWV0aG9kLlwiKTsgfVxuXG5mdW5jdGlvbiBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkobywgbWluTGVuKSB7IGlmICghbykgcmV0dXJuOyBpZiAodHlwZW9mIG8gPT09IFwic3RyaW5nXCIpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB2YXIgbiA9IE9iamVjdC5wcm90b3R5cGUudG9TdHJpbmcuY2FsbChvKS5zbGljZSg4LCAtMSk7IGlmIChuID09PSBcIk9iamVjdFwiICYmIG8uY29uc3RydWN0b3IpIG4gPSBvLmNvbnN0cnVjdG9yLm5hbWU7IGlmIChuID09PSBcIk1hcFwiIHx8IG4gPT09IFwiU2V0XCIpIHJldHVybiBBcnJheS5mcm9tKG8pOyBpZiAobiA9PT0gXCJBcmd1bWVudHNcIiB8fCAvXig/OlVpfEkpbnQoPzo4fDE2fDMyKSg/OkNsYW1wZWQpP0FycmF5JC8udGVzdChuKSkgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IH1cblxuZnVuY3Rpb24gX2FycmF5TGlrZVRvQXJyYXkoYXJyLCBsZW4pIHsgaWYgKGxlbiA9PSBudWxsIHx8IGxlbiA+IGFyci5sZW5ndGgpIGxlbiA9IGFyci5sZW5ndGg7IGZvciAodmFyIGkgPSAwLCBhcnIyID0gbmV3IEFycmF5KGxlbik7IGkgPCBsZW47IGkrKykgeyBhcnIyW2ldID0gYXJyW2ldOyB9IHJldHVybiBhcnIyOyB9XG5cbmZ1bmN0aW9uIF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHsgaWYgKHR5cGVvZiBTeW1ib2wgPT09IFwidW5kZWZpbmVkXCIgfHwgIShTeW1ib2wuaXRlcmF0b3IgaW4gT2JqZWN0KGFycikpKSByZXR1cm47IHZhciBfYXJyID0gW107IHZhciBfbiA9IHRydWU7IHZhciBfZCA9IGZhbHNlOyB2YXIgX2UgPSB1bmRlZmluZWQ7IHRyeSB7IGZvciAodmFyIF9pID0gYXJyW1N5bWJvbC5pdGVyYXRvcl0oKSwgX3M7ICEoX24gPSAoX3MgPSBfaS5uZXh0KCkpLmRvbmUpOyBfbiA9IHRydWUpIHsgX2Fyci5wdXNoKF9zLnZhbHVlKTsgaWYgKGkgJiYgX2Fyci5sZW5ndGggPT09IGkpIGJyZWFrOyB9IH0gY2F0Y2ggKGVycikgeyBfZCA9IHRydWU7IF9lID0gZXJyOyB9IGZpbmFsbHkgeyB0cnkgeyBpZiAoIV9uICYmIF9pW1wicmV0dXJuXCJdICE9IG51bGwpIF9pW1wicmV0dXJuXCJdKCk7IH0gZmluYWxseSB7IGlmIChfZCkgdGhyb3cgX2U7IH0gfSByZXR1cm4gX2FycjsgfVxuXG5mdW5jdGlvbiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB7IGlmIChBcnJheS5pc0FycmF5KGFycikpIHJldHVybiBhcnI7IH1cblxuLyoqXHJcbiAqIERldGVybWluZXMgd2hldGhlciBhIE1hdHJpeCBpcyB1cHBlciB0cmlhbmd1bGFyIE1hdHJpeCBvciBub3QuPGJyPjxicj5cclxuICogXHJcbiAqIFVwcGVyIHRyaWFuZ3VsYXIgTWF0cml4IGlzIGEgTWF0cml4IGluIHdoaWNoIGFsbCB0aGUgZW50cmllcyBiZWxvdyB0aGVcclxuICogbWFpbiBkaWFnb25hbCBhcmUgemVyby4gTm90ZSB0aGF0IGl0IGNhbiBiZSBhcHBsaWVkIHRvIGFueSBub24tc3F1YXJlIE1hdHJpeC48YnI+PGJyPlxyXG4gKiAgXHJcbiAqIFRoZSByZXN1bHQgaXMgY2FjaGVkLlxyXG4gKiBAbWVtYmVyb2YgTWF0cml4XHJcbiAqIEBpbnN0YW5jZVxyXG4gKiBAcGFyYW0ge251bWJlcn0gW2RpZ2l0PThdIC0gTnVtYmVyIG9mIHNpZ25pZmljYW50IGRpZ2l0c1xyXG4gKiBAcmV0dXJucyB7Ym9vbGVhbn0gUmV0dXJucyB0cnVlIGlmIHRoZSBNYXRyaXggaXMgdXBwZXIgdHJpYW5ndWxhclxyXG4gKi9cbmZ1bmN0aW9uIGlzVXBwZXJUcmlhbmd1bGFyKCkge1xuICB2YXIgZGlnaXQgPSBhcmd1bWVudHMubGVuZ3RoID4gMCAmJiBhcmd1bWVudHNbMF0gIT09IHVuZGVmaW5lZCA/IGFyZ3VtZW50c1swXSA6IHRoaXMuX2RpZ2l0O1xuXG4gIGlmICh0aGlzLl9pc1VwcGVyVHJpYW5ndWxhciAhPT0gdW5kZWZpbmVkKSB7XG4gICAgcmV0dXJuIHRoaXMuX2lzVXBwZXJUcmlhbmd1bGFyO1xuICB9XG5cbiAgdmFyIEVQU0lMT04gPSAxIC8gKE1hdGgucG93KDEwLCBkaWdpdCkgKiAyKTtcbiAgdmFyIEEgPSB0aGlzLl9tYXRyaXg7XG5cbiAgdmFyIF90aGlzJHNpemUgPSB0aGlzLnNpemUoKSxcbiAgICAgIF90aGlzJHNpemUyID0gX3NsaWNlZFRvQXJyYXkoX3RoaXMkc2l6ZSwgMiksXG4gICAgICByb3cgPSBfdGhpcyRzaXplMlswXSxcbiAgICAgIGNvbCA9IF90aGlzJHNpemUyWzFdO1xuXG4gIGlmIChyb3cgPT09IDApIHtcbiAgICAvLyBbXVxuICAgIHRoaXMuX2lzVXBwZXJUcmlhbmd1bGFyID0gdHJ1ZTtcbiAgICByZXR1cm4gdHJ1ZTtcbiAgfVxuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgcm93OyBpKyspIHtcbiAgICBmb3IgKHZhciBqID0gMDsgaiA8IGNvbDsgaisrKSB7XG4gICAgICBpZiAoaSA8PSBqKSB7XG4gICAgICAgIGNvbnRpbnVlO1xuICAgICAgfVxuXG4gICAgICBpZiAoTWF0aC5hYnMoQVtpXVtqXSkgPj0gRVBTSUxPTikge1xuICAgICAgICB0aGlzLl9pc1VwcGVyVHJpYW5ndWxhciA9IGZhbHNlO1xuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgICB9XG4gICAgfVxuICB9XG5cbiAgdGhpcy5faXNVcHBlclRyaWFuZ3VsYXIgPSB0cnVlO1xuICByZXR1cm4gdHJ1ZTtcbn1cblxuO1xubW9kdWxlLmV4cG9ydHMgPSBpc1VwcGVyVHJpYW5ndWxhcjsiLCJcInVzZSBzdHJpY3RcIjtcblxuZnVuY3Rpb24gX3NsaWNlZFRvQXJyYXkoYXJyLCBpKSB7IHJldHVybiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB8fCBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB8fCBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkoYXJyLCBpKSB8fCBfbm9uSXRlcmFibGVSZXN0KCk7IH1cblxuZnVuY3Rpb24gX25vbkl0ZXJhYmxlUmVzdCgpIHsgdGhyb3cgbmV3IFR5cGVFcnJvcihcIkludmFsaWQgYXR0ZW1wdCB0byBkZXN0cnVjdHVyZSBub24taXRlcmFibGUgaW5zdGFuY2UuXFxuSW4gb3JkZXIgdG8gYmUgaXRlcmFibGUsIG5vbi1hcnJheSBvYmplY3RzIG11c3QgaGF2ZSBhIFtTeW1ib2wuaXRlcmF0b3JdKCkgbWV0aG9kLlwiKTsgfVxuXG5mdW5jdGlvbiBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkobywgbWluTGVuKSB7IGlmICghbykgcmV0dXJuOyBpZiAodHlwZW9mIG8gPT09IFwic3RyaW5nXCIpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB2YXIgbiA9IE9iamVjdC5wcm90b3R5cGUudG9TdHJpbmcuY2FsbChvKS5zbGljZSg4LCAtMSk7IGlmIChuID09PSBcIk9iamVjdFwiICYmIG8uY29uc3RydWN0b3IpIG4gPSBvLmNvbnN0cnVjdG9yLm5hbWU7IGlmIChuID09PSBcIk1hcFwiIHx8IG4gPT09IFwiU2V0XCIpIHJldHVybiBBcnJheS5mcm9tKG8pOyBpZiAobiA9PT0gXCJBcmd1bWVudHNcIiB8fCAvXig/OlVpfEkpbnQoPzo4fDE2fDMyKSg/OkNsYW1wZWQpP0FycmF5JC8udGVzdChuKSkgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IH1cblxuZnVuY3Rpb24gX2FycmF5TGlrZVRvQXJyYXkoYXJyLCBsZW4pIHsgaWYgKGxlbiA9PSBudWxsIHx8IGxlbiA+IGFyci5sZW5ndGgpIGxlbiA9IGFyci5sZW5ndGg7IGZvciAodmFyIGkgPSAwLCBhcnIyID0gbmV3IEFycmF5KGxlbik7IGkgPCBsZW47IGkrKykgeyBhcnIyW2ldID0gYXJyW2ldOyB9IHJldHVybiBhcnIyOyB9XG5cbmZ1bmN0aW9uIF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHsgaWYgKHR5cGVvZiBTeW1ib2wgPT09IFwidW5kZWZpbmVkXCIgfHwgIShTeW1ib2wuaXRlcmF0b3IgaW4gT2JqZWN0KGFycikpKSByZXR1cm47IHZhciBfYXJyID0gW107IHZhciBfbiA9IHRydWU7IHZhciBfZCA9IGZhbHNlOyB2YXIgX2UgPSB1bmRlZmluZWQ7IHRyeSB7IGZvciAodmFyIF9pID0gYXJyW1N5bWJvbC5pdGVyYXRvcl0oKSwgX3M7ICEoX24gPSAoX3MgPSBfaS5uZXh0KCkpLmRvbmUpOyBfbiA9IHRydWUpIHsgX2Fyci5wdXNoKF9zLnZhbHVlKTsgaWYgKGkgJiYgX2Fyci5sZW5ndGggPT09IGkpIGJyZWFrOyB9IH0gY2F0Y2ggKGVycikgeyBfZCA9IHRydWU7IF9lID0gZXJyOyB9IGZpbmFsbHkgeyB0cnkgeyBpZiAoIV9uICYmIF9pW1wicmV0dXJuXCJdICE9IG51bGwpIF9pW1wicmV0dXJuXCJdKCk7IH0gZmluYWxseSB7IGlmIChfZCkgdGhyb3cgX2U7IH0gfSByZXR1cm4gX2FycjsgfVxuXG5mdW5jdGlvbiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB7IGlmIChBcnJheS5pc0FycmF5KGFycikpIHJldHVybiBhcnI7IH1cblxudmFyIF9yZXF1aXJlID0gcmVxdWlyZSgnLi4vLi4vRXJyb3InKSxcbiAgICBJTlZBTElEX01BVFJJWCA9IF9yZXF1aXJlLklOVkFMSURfTUFUUklYO1xuLyoqXHJcbiAqIENyZWF0ZXMgYSBjb3B5IG9mIE1hdHJpeC4gTm90ZSB0aGF0IGl0IHJlc2V0cyB0aGUgY2FjaGVkIGRhdGEuXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge01hdHJpeH0gQSAtIEFueSBNYXRyaXhcclxuICogQHJldHVybnMge01hdHJpeH0gQ29weSBvZiBBXHJcbiAqL1xuXG5cbmZ1bmN0aW9uIGNsb25lKEEpIHtcbiAgaWYgKCEoQSBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfTUFUUklYKTtcbiAgfVxuXG4gIHZhciBfQSRzaXplID0gQS5zaXplKCksXG4gICAgICBfQSRzaXplMiA9IF9zbGljZWRUb0FycmF5KF9BJHNpemUsIDIpLFxuICAgICAgcm93ID0gX0Ekc2l6ZTJbMF0sXG4gICAgICBjb2wgPSBfQSRzaXplMlsxXTtcblxuICB2YXIgbWF0cml4ID0gQS5fbWF0cml4O1xuICByZXR1cm4gdGhpcy5nZW5lcmF0ZShyb3csIGNvbCwgZnVuY3Rpb24gKGksIGopIHtcbiAgICByZXR1cm4gbWF0cml4W2ldW2pdO1xuICB9KTtcbn1cblxuO1xubW9kdWxlLmV4cG9ydHMgPSBjbG9uZTsiLCJcInVzZSBzdHJpY3RcIjtcblxuZnVuY3Rpb24gX3NsaWNlZFRvQXJyYXkoYXJyLCBpKSB7IHJldHVybiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB8fCBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB8fCBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkoYXJyLCBpKSB8fCBfbm9uSXRlcmFibGVSZXN0KCk7IH1cblxuZnVuY3Rpb24gX25vbkl0ZXJhYmxlUmVzdCgpIHsgdGhyb3cgbmV3IFR5cGVFcnJvcihcIkludmFsaWQgYXR0ZW1wdCB0byBkZXN0cnVjdHVyZSBub24taXRlcmFibGUgaW5zdGFuY2UuXFxuSW4gb3JkZXIgdG8gYmUgaXRlcmFibGUsIG5vbi1hcnJheSBvYmplY3RzIG11c3QgaGF2ZSBhIFtTeW1ib2wuaXRlcmF0b3JdKCkgbWV0aG9kLlwiKTsgfVxuXG5mdW5jdGlvbiBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkobywgbWluTGVuKSB7IGlmICghbykgcmV0dXJuOyBpZiAodHlwZW9mIG8gPT09IFwic3RyaW5nXCIpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB2YXIgbiA9IE9iamVjdC5wcm90b3R5cGUudG9TdHJpbmcuY2FsbChvKS5zbGljZSg4LCAtMSk7IGlmIChuID09PSBcIk9iamVjdFwiICYmIG8uY29uc3RydWN0b3IpIG4gPSBvLmNvbnN0cnVjdG9yLm5hbWU7IGlmIChuID09PSBcIk1hcFwiIHx8IG4gPT09IFwiU2V0XCIpIHJldHVybiBBcnJheS5mcm9tKG8pOyBpZiAobiA9PT0gXCJBcmd1bWVudHNcIiB8fCAvXig/OlVpfEkpbnQoPzo4fDE2fDMyKSg/OkNsYW1wZWQpP0FycmF5JC8udGVzdChuKSkgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IH1cblxuZnVuY3Rpb24gX2FycmF5TGlrZVRvQXJyYXkoYXJyLCBsZW4pIHsgaWYgKGxlbiA9PSBudWxsIHx8IGxlbiA+IGFyci5sZW5ndGgpIGxlbiA9IGFyci5sZW5ndGg7IGZvciAodmFyIGkgPSAwLCBhcnIyID0gbmV3IEFycmF5KGxlbik7IGkgPCBsZW47IGkrKykgeyBhcnIyW2ldID0gYXJyW2ldOyB9IHJldHVybiBhcnIyOyB9XG5cbmZ1bmN0aW9uIF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHsgaWYgKHR5cGVvZiBTeW1ib2wgPT09IFwidW5kZWZpbmVkXCIgfHwgIShTeW1ib2wuaXRlcmF0b3IgaW4gT2JqZWN0KGFycikpKSByZXR1cm47IHZhciBfYXJyID0gW107IHZhciBfbiA9IHRydWU7IHZhciBfZCA9IGZhbHNlOyB2YXIgX2UgPSB1bmRlZmluZWQ7IHRyeSB7IGZvciAodmFyIF9pID0gYXJyW1N5bWJvbC5pdGVyYXRvcl0oKSwgX3M7ICEoX24gPSAoX3MgPSBfaS5uZXh0KCkpLmRvbmUpOyBfbiA9IHRydWUpIHsgX2Fyci5wdXNoKF9zLnZhbHVlKTsgaWYgKGkgJiYgX2Fyci5sZW5ndGggPT09IGkpIGJyZWFrOyB9IH0gY2F0Y2ggKGVycikgeyBfZCA9IHRydWU7IF9lID0gZXJyOyB9IGZpbmFsbHkgeyB0cnkgeyBpZiAoIV9uICYmIF9pW1wicmV0dXJuXCJdICE9IG51bGwpIF9pW1wicmV0dXJuXCJdKCk7IH0gZmluYWxseSB7IGlmIChfZCkgdGhyb3cgX2U7IH0gfSByZXR1cm4gX2FycjsgfVxuXG5mdW5jdGlvbiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB7IGlmIChBcnJheS5pc0FycmF5KGFycikpIHJldHVybiBhcnI7IH1cblxudmFyIF9yZXF1aXJlID0gcmVxdWlyZSgnLi4vLi4vRXJyb3InKSxcbiAgICBJTlZBTElEX1JPV19DT0wgPSBfcmVxdWlyZS5JTlZBTElEX1JPV19DT0wsXG4gICAgT1ZFUkZMT1dfQ09MVU1OID0gX3JlcXVpcmUuT1ZFUkZMT1dfQ09MVU1OLFxuICAgIElOVkFMSURfTUFUUklYID0gX3JlcXVpcmUuSU5WQUxJRF9NQVRSSVg7XG4vKipcclxuICogR2V0cyB0aGUgY29sdW1uIG9mIGEgTWF0cml4IHdpdGggdmFsaWQgaW5kZXguXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge01hdHJpeH0gQSAtIEFueSBNYXRyaXhcclxuICogQHBhcmFtIHtudW1iZXJ9IGluZGV4IC0gQW55IHZhbGlkIGNvbHVtbiBpbmRleFxyXG4gKiBAcmV0dXJucyB7TWF0cml4fSBDb2x1bW4gb2YgQVxyXG4gKi9cblxuXG5mdW5jdGlvbiBjb2x1bW4oQSwgaW5kZXgpIHtcbiAgaWYgKCEoQSBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfTUFUUklYKTtcbiAgfVxuXG4gIGlmICghTnVtYmVyLmlzSW50ZWdlcihpbmRleCkgfHwgaW5kZXggPCAwKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfUk9XX0NPTCk7XG4gIH1cblxuICB2YXIgX0Ekc2l6ZSA9IEEuc2l6ZSgpLFxuICAgICAgX0Ekc2l6ZTIgPSBfc2xpY2VkVG9BcnJheShfQSRzaXplLCAyKSxcbiAgICAgIHIgPSBfQSRzaXplMlswXSxcbiAgICAgIGMgPSBfQSRzaXplMlsxXTtcblxuICBpZiAoaW5kZXggPj0gYykge1xuICAgIHRocm93IG5ldyBFcnJvcihPVkVSRkxPV19DT0xVTU4pO1xuICB9XG5cbiAgdmFyIG1hdHJpeCA9IEEuX21hdHJpeDtcbiAgcmV0dXJuIHRoaXMuZ2VuZXJhdGUociwgMSwgZnVuY3Rpb24gKGkpIHtcbiAgICByZXR1cm4gbWF0cml4W2ldW2luZGV4XTtcbiAgfSk7XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gY29sdW1uOyIsIlwidXNlIHN0cmljdFwiO1xuXG52YXIgTWF0cml4ID0gcmVxdWlyZSgnLi4vLi4nKTtcblxudmFyIGlzTnVtYmVyID0gcmVxdWlyZSgnLi4vLi4vdXRpbC9pc051bWJlcicpO1xuXG52YXIgX3JlcXVpcmUgPSByZXF1aXJlKCcuLi8uLi9FcnJvcicpLFxuICAgIElOVkFMSURfQVJSQVkgPSBfcmVxdWlyZS5JTlZBTElEX0FSUkFZLFxuICAgIEVYUEVDVEVEX0FSUkFZX09GX05VTUJFUlNfT1JfTUFUUklDRVMgPSBfcmVxdWlyZS5FWFBFQ1RFRF9BUlJBWV9PRl9OVU1CRVJTX09SX01BVFJJQ0VTLFxuICAgIElOVkFMSURfU1FVQVJFX01BVFJJWCA9IF9yZXF1aXJlLklOVkFMSURfU1FVQVJFX01BVFJJWDtcbi8qKlxyXG4gKiBHZW5lcmF0ZXMgZGlhZ29uYWwgTWF0cml4IGlmIHRoZSBhcmd1bWVudCBpcyBhbiBhcnJheSBvZiBudW1iZXJzLFxyXG4gKiBnZW5lcmF0ZXMgYmxvY2sgZGlhZ29uYWwgTWF0cml4IGlmIHRoZSBhcmd1bWVudCBpcyBhbiBhcnJheSBvZiBNYXRyaWNlcy5cclxuICogQG1lbWJlcm9mIE1hdHJpeFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7KG51bWJlcltdfE1hdHJpeFtdKX0gdmFsdWVzIC0gQXJyYXkgb2YgbnVtYmVycyBvciBNYXRyaWNlc1xyXG4gKiBAcmV0dXJucyB7TWF0cml4fSBCbG9jayBkaWFnb25hbCBNYXRyaXhcclxuICovXG5cblxuZnVuY3Rpb24gZGlhZyh2YWx1ZXMpIHtcbiAgaWYgKCFBcnJheS5pc0FycmF5KHZhbHVlcykpIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoSU5WQUxJRF9BUlJBWSk7XG4gIH1cblxuICB2YXIgYXJnc051bSA9IHZhbHVlcy5sZW5ndGg7XG4gIHZhciB2YXJpYW50O1xuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgYXJnc051bTsgaSsrKSB7XG4gICAgdmFyIGVudHJ5ID0gdmFsdWVzW2ldO1xuXG4gICAgaWYgKCFpc051bWJlcihlbnRyeSkgJiYgIShlbnRyeSBpbnN0YW5jZW9mIE1hdHJpeCkpIHtcbiAgICAgIHRocm93IG5ldyBFcnJvcihFWFBFQ1RFRF9BUlJBWV9PRl9OVU1CRVJTX09SX01BVFJJQ0VTKTtcbiAgICB9XG5cbiAgICBpZiAoaXNOdW1iZXIoZW50cnkpKSB7XG4gICAgICBpZiAoIXZhcmlhbnQpIHtcbiAgICAgICAgdmFyaWFudCA9ICdudW1iZXInO1xuICAgICAgICBjb250aW51ZTtcbiAgICAgIH1cblxuICAgICAgaWYgKHZhcmlhbnQgIT09ICdudW1iZXInKSB7XG4gICAgICAgIHRocm93IG5ldyBFcnJvcihFWFBFQ1RFRF9BUlJBWV9PRl9OVU1CRVJTX09SX01BVFJJQ0VTKTtcbiAgICAgIH1cbiAgICB9IGVsc2Uge1xuICAgICAgaWYgKCFlbnRyeS5pc1NxdWFyZSgpKSB7XG4gICAgICAgIHRocm93IG5ldyBFcnJvcihJTlZBTElEX1NRVUFSRV9NQVRSSVgpO1xuICAgICAgfVxuXG4gICAgICBpZiAoIXZhcmlhbnQpIHtcbiAgICAgICAgdmFyaWFudCA9ICdzcXVhcmUnO1xuICAgICAgICBjb250aW51ZTtcbiAgICAgIH1cblxuICAgICAgaWYgKHZhcmlhbnQgIT09ICdzcXVhcmUnKSB7XG4gICAgICAgIHRocm93IG5ldyBFcnJvcihFWFBFQ1RFRF9BUlJBWV9PRl9OVU1CRVJTX09SX01BVFJJQ0VTKTtcbiAgICAgIH1cbiAgICB9XG4gIH0gLy8gSEVSRTogdmFyaWFudCBzaG91bGQgYmUgZWl0aGVyICdudW1iZXInIG9yICdzcXVhcmUnXG5cblxuICBpZiAodmFyaWFudCA9PT0gJ251bWJlcicpIHtcbiAgICByZXR1cm4gTWF0cml4LmdlbmVyYXRlKGFyZ3NOdW0sIGFyZ3NOdW0sIGZ1bmN0aW9uIChpLCBqKSB7XG4gICAgICBpZiAoaSA9PT0gaikge1xuICAgICAgICByZXR1cm4gdmFsdWVzW2ldO1xuICAgICAgfVxuXG4gICAgICByZXR1cm4gMDtcbiAgICB9KTtcbiAgfSAvLyBHdWFyYW50ZWVkIHRoYXQgW3ZhbHVlc10gaXMgYSBsaXN0IG9mIHNxdWFyZSBtYXRyaWNlc1xuXG5cbiAgdmFyIHNpemUgPSAwO1xuICB2YXIgdGVtcCA9IG5ldyBBcnJheShhcmdzTnVtKTtcblxuICBmb3IgKHZhciBfaSA9IDA7IF9pIDwgYXJnc051bTsgX2krKykge1xuICAgIHZhciBfbGVuID0gdmFsdWVzW19pXS5zaXplKClbMF07XG5cbiAgICBzaXplICs9IF9sZW47XG4gICAgdGVtcFtfaV0gPSBfbGVuO1xuICB9XG5cbiAgdmFyIGlkeCA9IDA7XG4gIHZhciBzdGFydCA9IDA7XG4gIHZhciBsZW4gPSB0ZW1wW2lkeF07XG4gIHJldHVybiBNYXRyaXguZ2VuZXJhdGUoc2l6ZSwgc2l6ZSwgZnVuY3Rpb24gKGksIGopIHtcbiAgICBpZiAoaSAtIHN0YXJ0ID09PSBsZW4gJiYgaiAtIHN0YXJ0ID09PSBsZW4pIHtcbiAgICAgIHN0YXJ0ICs9IGxlbjtcbiAgICAgIGlkeCsrO1xuICAgIH1cblxuICAgIHZhciBpdGggPSBpIC0gc3RhcnQ7IC8vIGl0aCA8IDAgaWYgYmVsb3cgbWFpbiBkaWFnb25hbFxuXG4gICAgdmFyIGp0aCA9IGogLSBzdGFydDsgLy8ganRoIDwgMCBpZiBhYm92ZSBtYWluIGRpYWdvbmFsXG4gICAgLy8gc2tpcCAweDAgbWF0cmljZXNcblxuICAgIGxlbiA9IHRlbXBbaWR4XTtcblxuICAgIHdoaWxlIChsZW4gPT09IDApIHtcbiAgICAgIGlkeCsrO1xuICAgICAgbGVuID0gdGVtcFtpZHhdO1xuICAgIH1cblxuICAgIGlmIChpdGggPCBsZW4gJiYgaXRoID49IDAgJiYganRoIDwgbGVuICYmIGp0aCA+PSAwKSB7XG4gICAgICByZXR1cm4gdmFsdWVzW2lkeF0uX21hdHJpeFtpdGhdW2p0aF07XG4gICAgfVxuXG4gICAgcmV0dXJuIDA7XG4gIH0pO1xufVxuXG47XG5tb2R1bGUuZXhwb3J0cyA9IGRpYWc7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbmZ1bmN0aW9uIF9zbGljZWRUb0FycmF5KGFyciwgaSkgeyByZXR1cm4gX2FycmF5V2l0aEhvbGVzKGFycikgfHwgX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgfHwgX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KGFyciwgaSkgfHwgX25vbkl0ZXJhYmxlUmVzdCgpOyB9XG5cbmZ1bmN0aW9uIF9ub25JdGVyYWJsZVJlc3QoKSB7IHRocm93IG5ldyBUeXBlRXJyb3IoXCJJbnZhbGlkIGF0dGVtcHQgdG8gZGVzdHJ1Y3R1cmUgbm9uLWl0ZXJhYmxlIGluc3RhbmNlLlxcbkluIG9yZGVyIHRvIGJlIGl0ZXJhYmxlLCBub24tYXJyYXkgb2JqZWN0cyBtdXN0IGhhdmUgYSBbU3ltYm9sLml0ZXJhdG9yXSgpIG1ldGhvZC5cIik7IH1cblxuZnVuY3Rpb24gX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KG8sIG1pbkxlbikgeyBpZiAoIW8pIHJldHVybjsgaWYgKHR5cGVvZiBvID09PSBcInN0cmluZ1wiKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgdmFyIG4gPSBPYmplY3QucHJvdG90eXBlLnRvU3RyaW5nLmNhbGwobykuc2xpY2UoOCwgLTEpOyBpZiAobiA9PT0gXCJPYmplY3RcIiAmJiBvLmNvbnN0cnVjdG9yKSBuID0gby5jb25zdHJ1Y3Rvci5uYW1lOyBpZiAobiA9PT0gXCJNYXBcIiB8fCBuID09PSBcIlNldFwiKSByZXR1cm4gQXJyYXkuZnJvbShvKTsgaWYgKG4gPT09IFwiQXJndW1lbnRzXCIgfHwgL14oPzpVaXxJKW50KD86OHwxNnwzMikoPzpDbGFtcGVkKT9BcnJheSQvLnRlc3QobikpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB9XG5cbmZ1bmN0aW9uIF9hcnJheUxpa2VUb0FycmF5KGFyciwgbGVuKSB7IGlmIChsZW4gPT0gbnVsbCB8fCBsZW4gPiBhcnIubGVuZ3RoKSBsZW4gPSBhcnIubGVuZ3RoOyBmb3IgKHZhciBpID0gMCwgYXJyMiA9IG5ldyBBcnJheShsZW4pOyBpIDwgbGVuOyBpKyspIHsgYXJyMltpXSA9IGFycltpXTsgfSByZXR1cm4gYXJyMjsgfVxuXG5mdW5jdGlvbiBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB7IGlmICh0eXBlb2YgU3ltYm9sID09PSBcInVuZGVmaW5lZFwiIHx8ICEoU3ltYm9sLml0ZXJhdG9yIGluIE9iamVjdChhcnIpKSkgcmV0dXJuOyB2YXIgX2FyciA9IFtdOyB2YXIgX24gPSB0cnVlOyB2YXIgX2QgPSBmYWxzZTsgdmFyIF9lID0gdW5kZWZpbmVkOyB0cnkgeyBmb3IgKHZhciBfaSA9IGFycltTeW1ib2wuaXRlcmF0b3JdKCksIF9zOyAhKF9uID0gKF9zID0gX2kubmV4dCgpKS5kb25lKTsgX24gPSB0cnVlKSB7IF9hcnIucHVzaChfcy52YWx1ZSk7IGlmIChpICYmIF9hcnIubGVuZ3RoID09PSBpKSBicmVhazsgfSB9IGNhdGNoIChlcnIpIHsgX2QgPSB0cnVlOyBfZSA9IGVycjsgfSBmaW5hbGx5IHsgdHJ5IHsgaWYgKCFfbiAmJiBfaVtcInJldHVyblwiXSAhPSBudWxsKSBfaVtcInJldHVyblwiXSgpOyB9IGZpbmFsbHkgeyBpZiAoX2QpIHRocm93IF9lOyB9IH0gcmV0dXJuIF9hcnI7IH1cblxuZnVuY3Rpb24gX2FycmF5V2l0aEhvbGVzKGFycikgeyBpZiAoQXJyYXkuaXNBcnJheShhcnIpKSByZXR1cm4gYXJyOyB9XG5cbnZhciBfcmVxdWlyZSA9IHJlcXVpcmUoJy4uLy4uL0Vycm9yJyksXG4gICAgSU5WQUxJRF9NQVRSSVggPSBfcmVxdWlyZS5JTlZBTElEX01BVFJJWDtcbi8qKlxyXG4gKiBUaGlzIGNhbGxiYWNrIGFwcGxpZXMgb24gZWFjaCBlbnRyeSBvZiBhIE1hdHJpeFxyXG4gKiBAY2FsbGJhY2sgZW50cnlDYWxsYmFja1xyXG4gKiBAcGFyYW0ge251bWJlcn0gZW50cnkgLSBFbnRyeSBvZiBhIE1hdHJpeFxyXG4gKiBAcmV0dXJucyB7bnVtYmVyfSBOZXcgZW50cnkgdmFsdWVcclxuICovXG5cbi8qKlxyXG4gKiBBcHBseXMgYSBmdW5jdGlvbiBvdmVyIGVhY2ggZW50cnkgb2YgYSBNYXRyaXggYW5kIHJldHVybnNcclxuICogYSBuZXcgY29weSBvZiB0aGUgbmV3IE1hdHJpeC5cclxuICogQG1lbWJlcm9mIE1hdHJpeFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7TWF0cml4fSBBIC0gQW55IE1hdHJpeFxyXG4gKiBAcGFyYW0ge2VudHJ5Q2FsbGJhY2t9IGNiIC0gQ2FsbGJhY2sgZnVuY3Rpb24gd2hpY2ggYXBwbGllcyBvbiBlYWNoIGVudHJ5IG9mIEFcclxuICogQHJldHVybnMge01hdHJpeH0gQSBjb3B5IG9mIG5ldyBNYXRyaXhcclxuICovXG5cblxuZnVuY3Rpb24gZWxlbWVudHdpc2UoQSwgY2IpIHtcbiAgaWYgKCEoQSBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfTUFUUklYKTtcbiAgfVxuXG4gIHZhciBfQSRzaXplID0gQS5zaXplKCksXG4gICAgICBfQSRzaXplMiA9IF9zbGljZWRUb0FycmF5KF9BJHNpemUsIDIpLFxuICAgICAgcm93ID0gX0Ekc2l6ZTJbMF0sXG4gICAgICBjb2wgPSBfQSRzaXplMlsxXTtcblxuICB2YXIgbWF0cml4ID0gQS5fbWF0cml4O1xuICByZXR1cm4gdGhpcy5nZW5lcmF0ZShyb3csIGNvbCwgZnVuY3Rpb24gKGksIGopIHtcbiAgICByZXR1cm4gY2IobWF0cml4W2ldW2pdKTtcbiAgfSk7XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gZWxlbWVudHdpc2U7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbmZ1bmN0aW9uIF9zbGljZWRUb0FycmF5KGFyciwgaSkgeyByZXR1cm4gX2FycmF5V2l0aEhvbGVzKGFycikgfHwgX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgfHwgX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KGFyciwgaSkgfHwgX25vbkl0ZXJhYmxlUmVzdCgpOyB9XG5cbmZ1bmN0aW9uIF9ub25JdGVyYWJsZVJlc3QoKSB7IHRocm93IG5ldyBUeXBlRXJyb3IoXCJJbnZhbGlkIGF0dGVtcHQgdG8gZGVzdHJ1Y3R1cmUgbm9uLWl0ZXJhYmxlIGluc3RhbmNlLlxcbkluIG9yZGVyIHRvIGJlIGl0ZXJhYmxlLCBub24tYXJyYXkgb2JqZWN0cyBtdXN0IGhhdmUgYSBbU3ltYm9sLml0ZXJhdG9yXSgpIG1ldGhvZC5cIik7IH1cblxuZnVuY3Rpb24gX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KG8sIG1pbkxlbikgeyBpZiAoIW8pIHJldHVybjsgaWYgKHR5cGVvZiBvID09PSBcInN0cmluZ1wiKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgdmFyIG4gPSBPYmplY3QucHJvdG90eXBlLnRvU3RyaW5nLmNhbGwobykuc2xpY2UoOCwgLTEpOyBpZiAobiA9PT0gXCJPYmplY3RcIiAmJiBvLmNvbnN0cnVjdG9yKSBuID0gby5jb25zdHJ1Y3Rvci5uYW1lOyBpZiAobiA9PT0gXCJNYXBcIiB8fCBuID09PSBcIlNldFwiKSByZXR1cm4gQXJyYXkuZnJvbShvKTsgaWYgKG4gPT09IFwiQXJndW1lbnRzXCIgfHwgL14oPzpVaXxJKW50KD86OHwxNnwzMikoPzpDbGFtcGVkKT9BcnJheSQvLnRlc3QobikpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB9XG5cbmZ1bmN0aW9uIF9hcnJheUxpa2VUb0FycmF5KGFyciwgbGVuKSB7IGlmIChsZW4gPT0gbnVsbCB8fCBsZW4gPiBhcnIubGVuZ3RoKSBsZW4gPSBhcnIubGVuZ3RoOyBmb3IgKHZhciBpID0gMCwgYXJyMiA9IG5ldyBBcnJheShsZW4pOyBpIDwgbGVuOyBpKyspIHsgYXJyMltpXSA9IGFycltpXTsgfSByZXR1cm4gYXJyMjsgfVxuXG5mdW5jdGlvbiBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB7IGlmICh0eXBlb2YgU3ltYm9sID09PSBcInVuZGVmaW5lZFwiIHx8ICEoU3ltYm9sLml0ZXJhdG9yIGluIE9iamVjdChhcnIpKSkgcmV0dXJuOyB2YXIgX2FyciA9IFtdOyB2YXIgX24gPSB0cnVlOyB2YXIgX2QgPSBmYWxzZTsgdmFyIF9lID0gdW5kZWZpbmVkOyB0cnkgeyBmb3IgKHZhciBfaSA9IGFycltTeW1ib2wuaXRlcmF0b3JdKCksIF9zOyAhKF9uID0gKF9zID0gX2kubmV4dCgpKS5kb25lKTsgX24gPSB0cnVlKSB7IF9hcnIucHVzaChfcy52YWx1ZSk7IGlmIChpICYmIF9hcnIubGVuZ3RoID09PSBpKSBicmVhazsgfSB9IGNhdGNoIChlcnIpIHsgX2QgPSB0cnVlOyBfZSA9IGVycjsgfSBmaW5hbGx5IHsgdHJ5IHsgaWYgKCFfbiAmJiBfaVtcInJldHVyblwiXSAhPSBudWxsKSBfaVtcInJldHVyblwiXSgpOyB9IGZpbmFsbHkgeyBpZiAoX2QpIHRocm93IF9lOyB9IH0gcmV0dXJuIF9hcnI7IH1cblxuZnVuY3Rpb24gX2FycmF5V2l0aEhvbGVzKGFycikgeyBpZiAoQXJyYXkuaXNBcnJheShhcnIpKSByZXR1cm4gYXJyOyB9XG5cbnZhciBfcmVxdWlyZSA9IHJlcXVpcmUoJy4uLy4uL0Vycm9yJyksXG4gICAgSU5WQUxJRF9ST1dfQ09MID0gX3JlcXVpcmUuSU5WQUxJRF9ST1dfQ09MLFxuICAgIE9WRVJGTE9XX0lOREVYID0gX3JlcXVpcmUuT1ZFUkZMT1dfSU5ERVg7XG4vKipcclxuICogR2V0cyB0aGUgZW50cnkgb2YgYSBNYXRyaXguXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQGluc3RhbmNlXHJcbiAqIEBwYXJhbSB7bnVtYmVyfSByb3cgLSBBbnkgdmFsaWQgcm93IGluZGV4XHJcbiAqIEBwYXJhbSB7bnVtYmVyfSBjb2wgLSBBbnkgdmFsaWQgY29sdW1uIGluZGV4XHJcbiAqIEByZXR1cm5zIHtudW1iZXJ9IEVudHJ5IG9mIHRoZSBNYXRyaXhcclxuICovXG5cblxuZnVuY3Rpb24gZW50cnkocm93LCBjb2wpIHtcbiAgaWYgKCFOdW1iZXIuaXNJbnRlZ2VyKHJvdykgfHwgcm93IDwgMCB8fCAhTnVtYmVyLmlzSW50ZWdlcihjb2wpIHx8IGNvbCA8IDApIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoSU5WQUxJRF9ST1dfQ09MKTtcbiAgfVxuXG4gIHZhciBBID0gdGhpcy5fbWF0cml4O1xuXG4gIHZhciBfdGhpcyRzaXplID0gdGhpcy5zaXplKCksXG4gICAgICBfdGhpcyRzaXplMiA9IF9zbGljZWRUb0FycmF5KF90aGlzJHNpemUsIDIpLFxuICAgICAgciA9IF90aGlzJHNpemUyWzBdLFxuICAgICAgYyA9IF90aGlzJHNpemUyWzFdO1xuXG4gIGlmIChyb3cgPj0gciB8fCBjb2wgPj0gYykge1xuICAgIHRocm93IG5ldyBFcnJvcihPVkVSRkxPV19JTkRFWCk7XG4gIH1cblxuICByZXR1cm4gQVtyb3ddW2NvbF07XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gZW50cnk7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbnZhciBlbXB0eSA9IHJlcXVpcmUoJy4uLy4uL3V0aWwvZW1wdHknKTtcbi8qKlxyXG4gKiBUaGlzIGNhbGxiYWNrIGdlbmVyYXRlcyBlYWNoIGVudHJ5IG9mIGEgTWF0cml4XHJcbiAqIEBjYWxsYmFjayBnZW5lcmF0ZUNhbGxiYWNrXHJcbiAqIEBwYXJhbSB7bnVtYmVyfSBpIC0gVGhlIGktdGggcm93IG9mIE1hdHJpeCBcclxuICogQHBhcmFtIHtudW1iZXJ9IGogLSBUaGUgai10aCBjb2x1bW4gb2YgTWF0cml4IFxyXG4gKiBAcmV0dXJucyB7bnVtYmVyfSBFbnRyeSBvZiBNYXRyaXhcclxuICovXG5cbi8qKlxyXG4gKiBHZW5lcmF0ZXMgYSBNYXRyaXggd2hpY2ggZW50cmllcyBhcmUgdGhlIHJldHVybmVkIHZhbHVlIG9mIGNhbGxiYWNrIGZ1bmN0aW9uLlxyXG4gKiBAbWVtYmVyb2YgTWF0cml4XHJcbiAqIEBzdGF0aWNcclxuICogQHBhcmFtIHtudW1iZXJ9IHJvdyAtIE51bWJlciBvZiByb3dzIG9mIE1hdHJpeFxyXG4gKiBAcGFyYW0ge251bWJlcn0gY29sIC0gTnVtYmVyIG9mIGNvbHVtbnMgb2YgTWF0cml4XHJcbiAqIEBwYXJhbSB7Z2VuZXJhdGVDYWxsYmFja30gY2IgLSBDYWxsYmFjayBmdW5jdGlvbiB3aGljaCB0YWtlcyByb3cgYW5kIGNvbHVtbiBhcyBhcmd1bWVudHNcclxuICogYW5kIGdlbmVyYXRlcyB0aGUgY29ycmVzcG9uZGluZyBlbnRyeVxyXG4gKiBAcmV0dXJucyB7TWF0cml4fSAtIEdlbmVyYXRlZCBNYXRyaXhcclxuICovXG5cblxuZnVuY3Rpb24gZ2VuZXJhdGUocm93LCBjb2wsIGNiKSB7XG4gIHZhciBtYXRyaXggPSBlbXB0eShyb3csIGNvbCk7XG5cbiAgaWYgKHJvdyA9PT0gMCB8fCBjb2wgPT09IDApIHtcbiAgICByZXR1cm4gbmV3IHRoaXMoW10pO1xuICB9XG5cbiAgZm9yICh2YXIgaSA9IDA7IGkgPCByb3c7IGkrKykge1xuICAgIGZvciAodmFyIGogPSAwOyBqIDwgY29sOyBqKyspIHtcbiAgICAgIG1hdHJpeFtpXVtqXSA9IGNiKGksIGopO1xuICAgIH1cbiAgfVxuXG4gIHJldHVybiBuZXcgdGhpcyhtYXRyaXgpO1xufVxuXG47XG5tb2R1bGUuZXhwb3J0cyA9IGdlbmVyYXRlOyIsIlwidXNlIHN0cmljdFwiO1xuXG5mdW5jdGlvbiBfc2xpY2VkVG9BcnJheShhcnIsIGkpIHsgcmV0dXJuIF9hcnJheVdpdGhIb2xlcyhhcnIpIHx8IF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHx8IF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShhcnIsIGkpIHx8IF9ub25JdGVyYWJsZVJlc3QoKTsgfVxuXG5mdW5jdGlvbiBfbm9uSXRlcmFibGVSZXN0KCkgeyB0aHJvdyBuZXcgVHlwZUVycm9yKFwiSW52YWxpZCBhdHRlbXB0IHRvIGRlc3RydWN0dXJlIG5vbi1pdGVyYWJsZSBpbnN0YW5jZS5cXG5JbiBvcmRlciB0byBiZSBpdGVyYWJsZSwgbm9uLWFycmF5IG9iamVjdHMgbXVzdCBoYXZlIGEgW1N5bWJvbC5pdGVyYXRvcl0oKSBtZXRob2QuXCIpOyB9XG5cbmZ1bmN0aW9uIF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShvLCBtaW5MZW4pIHsgaWYgKCFvKSByZXR1cm47IGlmICh0eXBlb2YgbyA9PT0gXCJzdHJpbmdcIikgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IHZhciBuID0gT2JqZWN0LnByb3RvdHlwZS50b1N0cmluZy5jYWxsKG8pLnNsaWNlKDgsIC0xKTsgaWYgKG4gPT09IFwiT2JqZWN0XCIgJiYgby5jb25zdHJ1Y3RvcikgbiA9IG8uY29uc3RydWN0b3IubmFtZTsgaWYgKG4gPT09IFwiTWFwXCIgfHwgbiA9PT0gXCJTZXRcIikgcmV0dXJuIEFycmF5LmZyb20obyk7IGlmIChuID09PSBcIkFyZ3VtZW50c1wiIHx8IC9eKD86VWl8SSludCg/Ojh8MTZ8MzIpKD86Q2xhbXBlZCk/QXJyYXkkLy50ZXN0KG4pKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgfVxuXG5mdW5jdGlvbiBfYXJyYXlMaWtlVG9BcnJheShhcnIsIGxlbikgeyBpZiAobGVuID09IG51bGwgfHwgbGVuID4gYXJyLmxlbmd0aCkgbGVuID0gYXJyLmxlbmd0aDsgZm9yICh2YXIgaSA9IDAsIGFycjIgPSBuZXcgQXJyYXkobGVuKTsgaSA8IGxlbjsgaSsrKSB7IGFycjJbaV0gPSBhcnJbaV07IH0gcmV0dXJuIGFycjI7IH1cblxuZnVuY3Rpb24gX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgeyBpZiAodHlwZW9mIFN5bWJvbCA9PT0gXCJ1bmRlZmluZWRcIiB8fCAhKFN5bWJvbC5pdGVyYXRvciBpbiBPYmplY3QoYXJyKSkpIHJldHVybjsgdmFyIF9hcnIgPSBbXTsgdmFyIF9uID0gdHJ1ZTsgdmFyIF9kID0gZmFsc2U7IHZhciBfZSA9IHVuZGVmaW5lZDsgdHJ5IHsgZm9yICh2YXIgX2kgPSBhcnJbU3ltYm9sLml0ZXJhdG9yXSgpLCBfczsgIShfbiA9IChfcyA9IF9pLm5leHQoKSkuZG9uZSk7IF9uID0gdHJ1ZSkgeyBfYXJyLnB1c2goX3MudmFsdWUpOyBpZiAoaSAmJiBfYXJyLmxlbmd0aCA9PT0gaSkgYnJlYWs7IH0gfSBjYXRjaCAoZXJyKSB7IF9kID0gdHJ1ZTsgX2UgPSBlcnI7IH0gZmluYWxseSB7IHRyeSB7IGlmICghX24gJiYgX2lbXCJyZXR1cm5cIl0gIT0gbnVsbCkgX2lbXCJyZXR1cm5cIl0oKTsgfSBmaW5hbGx5IHsgaWYgKF9kKSB0aHJvdyBfZTsgfSB9IHJldHVybiBfYXJyOyB9XG5cbmZ1bmN0aW9uIF9hcnJheVdpdGhIb2xlcyhhcnIpIHsgaWYgKEFycmF5LmlzQXJyYXkoYXJyKSkgcmV0dXJuIGFycjsgfVxuXG52YXIgX3JlcXVpcmUgPSByZXF1aXJlKCcuLi8uLi9FcnJvcicpLFxuICAgIElOVkFMSURfTUFUUklYID0gX3JlcXVpcmUuSU5WQUxJRF9NQVRSSVg7XG4vKipcclxuICogR2V0cyB0aGUgZW50cmllcyBvbiB0aGUgbWFpbiBkaWFnb25hbC5cclxuICogQG1lbWJlcm9mIE1hdHJpeFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7TWF0cml4fSBBIC0gQW55IE1hdHJpeFxyXG4gKiBAcmV0dXJucyB7bnVtYmVyW119IEFycmF5IG9mIGVudHJpZXMgb2YgQSBvbiB0aGUgbWFpbiBkaWFnb25hbFxyXG4gKi9cblxuXG5mdW5jdGlvbiBnZXREaWFnKEEpIHtcbiAgaWYgKCEoQSBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfTUFUUklYKTtcbiAgfVxuXG4gIHZhciBfQSRzaXplID0gQS5zaXplKCksXG4gICAgICBfQSRzaXplMiA9IF9zbGljZWRUb0FycmF5KF9BJHNpemUsIDIpLFxuICAgICAgcm93ID0gX0Ekc2l6ZTJbMF0sXG4gICAgICBjb2wgPSBfQSRzaXplMlsxXTtcblxuICB2YXIgc2l6ZSA9IE1hdGgubWluKHJvdywgY29sKTtcbiAgdmFyIG1hdHJpeCA9IEEuX21hdHJpeDtcbiAgdmFyIGRpYWdzID0gbmV3IEFycmF5KHNpemUpO1xuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgc2l6ZTsgaSsrKSB7XG4gICAgZGlhZ3NbaV0gPSBtYXRyaXhbaV1baV07XG4gIH1cblxuICByZXR1cm4gZGlhZ3M7XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gZ2V0RGlhZzsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIEdlbmVyYXRlcyBhIHJhbmRvbSBNYXRyaXguXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge251bWJlcn0gcm93IC0gTnVtYmVyIG9mIHJvd3Mgb2YgYSBNYXRyaXhcclxuICogQHBhcmFtIHtudW1iZXJ9IGNvbCAtIE51bWJlciBvZiBjb2x1bW5zIG9mIGEgTWF0cml4XHJcbiAqIEBwYXJhbSB7bnVtYmVyfSBtaW4gLSBMb3dlciBib3VuZCBvZiBlYWNoIGVudHJ5XHJcbiAqIEBwYXJhbSB7bnVtYmVyfSBtYXggLSBVcHBlciBib3VuZCBvZiBlYWNoIGVudHJ5XHJcbiAqIEBwYXJhbSB7bnVtYmVyfSB0b0ZpeGVkIC0gTnVtYmVyIG9mIGRlY2ltYWwgcGxhY2VzXHJcbiAqIEByZXR1cm5zIHtNYXRyaXh9IEdlbmVyYXRlZCByYW5kb20gTWF0cml4XHJcbiAqL1xuZnVuY3Rpb24gZ2V0UmFuZG9tTWF0cml4KHJvdywgY29sKSB7XG4gIHZhciBtaW4gPSBhcmd1bWVudHMubGVuZ3RoID4gMiAmJiBhcmd1bWVudHNbMl0gIT09IHVuZGVmaW5lZCA/IGFyZ3VtZW50c1syXSA6IDA7XG4gIHZhciBtYXggPSBhcmd1bWVudHMubGVuZ3RoID4gMyAmJiBhcmd1bWVudHNbM10gIT09IHVuZGVmaW5lZCA/IGFyZ3VtZW50c1szXSA6IDE7XG4gIHZhciB0b0ZpeGVkID0gYXJndW1lbnRzLmxlbmd0aCA+IDQgJiYgYXJndW1lbnRzWzRdICE9PSB1bmRlZmluZWQgPyBhcmd1bWVudHNbNF0gOiAwO1xuICByZXR1cm4gdGhpcy5nZW5lcmF0ZShyb3csIGNvbCwgZnVuY3Rpb24gKCkge1xuICAgIHJldHVybiBOdW1iZXIucGFyc2VGbG9hdCgoTWF0aC5yYW5kb20oKSAqIChtYXggLSBtaW4pICsgbWluKS50b0ZpeGVkKHRvRml4ZWQpKTtcbiAgfSk7XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gZ2V0UmFuZG9tTWF0cml4OyIsIlwidXNlIHN0cmljdFwiO1xuXG4vKipcclxuICogR2VuZXJhdGVzIGlkZW50aXR5IE1hdHJpeCB3aXRoIGdpdmVuIHNpemUuXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge251bWJlcn0gc2l6ZSAtIFRoZSBzaXplIG9mIE1hdHJpeFxyXG4gKiBAcmV0dXJucyB7TWF0cml4fSBJZGVudGl0eSBNYXRyaXhcclxuICovXG5mdW5jdGlvbiBpZGVudGl0eShzaXplKSB7XG4gIHJldHVybiB0aGlzLmdlbmVyYXRlKHNpemUsIHNpemUsIGZ1bmN0aW9uIChpLCBqKSB7XG4gICAgaWYgKGkgPT09IGopIHtcbiAgICAgIHJldHVybiAxO1xuICAgIH1cblxuICAgIHJldHVybiAwO1xuICB9KTtcbn1cblxuO1xubW9kdWxlLmV4cG9ydHMgPSBpZGVudGl0eTsiLCJcInVzZSBzdHJpY3RcIjtcblxuZnVuY3Rpb24gX3NsaWNlZFRvQXJyYXkoYXJyLCBpKSB7IHJldHVybiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB8fCBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB8fCBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkoYXJyLCBpKSB8fCBfbm9uSXRlcmFibGVSZXN0KCk7IH1cblxuZnVuY3Rpb24gX25vbkl0ZXJhYmxlUmVzdCgpIHsgdGhyb3cgbmV3IFR5cGVFcnJvcihcIkludmFsaWQgYXR0ZW1wdCB0byBkZXN0cnVjdHVyZSBub24taXRlcmFibGUgaW5zdGFuY2UuXFxuSW4gb3JkZXIgdG8gYmUgaXRlcmFibGUsIG5vbi1hcnJheSBvYmplY3RzIG11c3QgaGF2ZSBhIFtTeW1ib2wuaXRlcmF0b3JdKCkgbWV0aG9kLlwiKTsgfVxuXG5mdW5jdGlvbiBfdW5zdXBwb3J0ZWRJdGVyYWJsZVRvQXJyYXkobywgbWluTGVuKSB7IGlmICghbykgcmV0dXJuOyBpZiAodHlwZW9mIG8gPT09IFwic3RyaW5nXCIpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB2YXIgbiA9IE9iamVjdC5wcm90b3R5cGUudG9TdHJpbmcuY2FsbChvKS5zbGljZSg4LCAtMSk7IGlmIChuID09PSBcIk9iamVjdFwiICYmIG8uY29uc3RydWN0b3IpIG4gPSBvLmNvbnN0cnVjdG9yLm5hbWU7IGlmIChuID09PSBcIk1hcFwiIHx8IG4gPT09IFwiU2V0XCIpIHJldHVybiBBcnJheS5mcm9tKG8pOyBpZiAobiA9PT0gXCJBcmd1bWVudHNcIiB8fCAvXig/OlVpfEkpbnQoPzo4fDE2fDMyKSg/OkNsYW1wZWQpP0FycmF5JC8udGVzdChuKSkgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IH1cblxuZnVuY3Rpb24gX2FycmF5TGlrZVRvQXJyYXkoYXJyLCBsZW4pIHsgaWYgKGxlbiA9PSBudWxsIHx8IGxlbiA+IGFyci5sZW5ndGgpIGxlbiA9IGFyci5sZW5ndGg7IGZvciAodmFyIGkgPSAwLCBhcnIyID0gbmV3IEFycmF5KGxlbik7IGkgPCBsZW47IGkrKykgeyBhcnIyW2ldID0gYXJyW2ldOyB9IHJldHVybiBhcnIyOyB9XG5cbmZ1bmN0aW9uIF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHsgaWYgKHR5cGVvZiBTeW1ib2wgPT09IFwidW5kZWZpbmVkXCIgfHwgIShTeW1ib2wuaXRlcmF0b3IgaW4gT2JqZWN0KGFycikpKSByZXR1cm47IHZhciBfYXJyID0gW107IHZhciBfbiA9IHRydWU7IHZhciBfZCA9IGZhbHNlOyB2YXIgX2UgPSB1bmRlZmluZWQ7IHRyeSB7IGZvciAodmFyIF9pID0gYXJyW1N5bWJvbC5pdGVyYXRvcl0oKSwgX3M7ICEoX24gPSAoX3MgPSBfaS5uZXh0KCkpLmRvbmUpOyBfbiA9IHRydWUpIHsgX2Fyci5wdXNoKF9zLnZhbHVlKTsgaWYgKGkgJiYgX2Fyci5sZW5ndGggPT09IGkpIGJyZWFrOyB9IH0gY2F0Y2ggKGVycikgeyBfZCA9IHRydWU7IF9lID0gZXJyOyB9IGZpbmFsbHkgeyB0cnkgeyBpZiAoIV9uICYmIF9pW1wicmV0dXJuXCJdICE9IG51bGwpIF9pW1wicmV0dXJuXCJdKCk7IH0gZmluYWxseSB7IGlmIChfZCkgdGhyb3cgX2U7IH0gfSByZXR1cm4gX2FycjsgfVxuXG5mdW5jdGlvbiBfYXJyYXlXaXRoSG9sZXMoYXJyKSB7IGlmIChBcnJheS5pc0FycmF5KGFycikpIHJldHVybiBhcnI7IH1cblxudmFyIF9yZXF1aXJlID0gcmVxdWlyZSgnLi4vLi4vRXJyb3InKSxcbiAgICBJTlZBTElEX01BVFJJWCA9IF9yZXF1aXJlLklOVkFMSURfTUFUUklYO1xuLyoqXHJcbiAqIERldGVybWluZXMgd2hldGhlciB0d28gTWF0cmljZXMgYXJlIGNvbnNpZGVyZWQgYXMgZXF1YWwuPGJyPjxicj5cclxuICogXHJcbiAqIFRoZSB0ZXN0IGNyaXRlcmlvbiBpcyBNYXRoLmFicyh4IC0geSkgPCAxIC8gKDEwICoqIGRpZ2l0ICogMikuXHJcbiAqIEZvciBkZWZhdWx0IHZhbHVlIDUsIGl0IHNob3VsZCBiZSA1ZS01LlxyXG4gKiBUaGF0IG1lYW5zIGlmIHRoZSBkaWZmZXJlbmNlIG9mIHR3byBudW1iZXJzIGlzIGxlc3MgdGhhbiA1ZS01LFxyXG4gKiB0aGV5IGFyZSBjb25zaWRlcmVkIGFzIHNhbWUgdmFsdWUuXHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge01hdHJpeH0gQSAtIEFueSBNYXRyaXhcclxuICogQHBhcmFtIHtNYXRyaXh9IEIgLSBBbnkgTWF0cml4XHJcbiAqIEBwYXJhbSB7bnVtYmVyfSBkaWdpdCAtIE51bWJlciBvZiBzaWduaWZpY2FudCBkaWdpdHNcclxuICogQHJldHVybnMge2Jvb2xlYW59IFJldHVybnMgdHJ1ZSBpZiB0d28gTWF0cmljZXMgYXJlIGNvbnNpZGVyZWQgYXMgc2FtZVxyXG4gKi9cblxuXG5mdW5jdGlvbiBpc0VxdWFsKEEsIEIpIHtcbiAgdmFyIGRpZ2l0ID0gYXJndW1lbnRzLmxlbmd0aCA+IDIgJiYgYXJndW1lbnRzWzJdICE9PSB1bmRlZmluZWQgPyBhcmd1bWVudHNbMl0gOiA1O1xuXG4gIGlmICghKEEgaW5zdGFuY2VvZiB0aGlzKSB8fCAhKEIgaW5zdGFuY2VvZiB0aGlzKSkge1xuICAgIHRocm93IG5ldyBFcnJvcihJTlZBTElEX01BVFJJWCk7XG4gIH1cblxuICB2YXIgX0Ekc2l6ZSA9IEEuc2l6ZSgpLFxuICAgICAgX0Ekc2l6ZTIgPSBfc2xpY2VkVG9BcnJheShfQSRzaXplLCAyKSxcbiAgICAgIEFyb3cgPSBfQSRzaXplMlswXSxcbiAgICAgIEFjb2wgPSBfQSRzaXplMlsxXTtcblxuICB2YXIgX0Ikc2l6ZSA9IEIuc2l6ZSgpLFxuICAgICAgX0Ikc2l6ZTIgPSBfc2xpY2VkVG9BcnJheShfQiRzaXplLCAyKSxcbiAgICAgIEJyb3cgPSBfQiRzaXplMlswXSxcbiAgICAgIEJjb2wgPSBfQiRzaXplMlsxXTtcblxuICBpZiAoQXJvdyAhPT0gQnJvdyB8fCBBY29sICE9PSBCY29sKSB7XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG5cbiAgdmFyIEVQSVNJTE9OID0gMSAvIChNYXRoLnBvdygxMCwgZGlnaXQpICogMik7XG4gIHZhciBtYXRyaXhBID0gQS5fbWF0cml4O1xuICB2YXIgbWF0cml4QiA9IEIuX21hdHJpeDtcblxuICBmb3IgKHZhciBpID0gMDsgaSA8IEFyb3c7IGkrKykge1xuICAgIGZvciAodmFyIGogPSAwOyBqIDwgQWNvbDsgaisrKSB7XG4gICAgICBpZiAoTWF0aC5hYnMobWF0cml4QVtpXVtqXSAtIG1hdHJpeEJbaV1bal0pID49IEVQSVNJTE9OKSB7XG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICAgIH1cbiAgICB9XG4gIH1cblxuICByZXR1cm4gdHJ1ZTtcbn1cblxuO1xubW9kdWxlLmV4cG9ydHMgPSBpc0VxdWFsOyIsIlwidXNlIHN0cmljdFwiO1xuXG5mdW5jdGlvbiBfc2xpY2VkVG9BcnJheShhcnIsIGkpIHsgcmV0dXJuIF9hcnJheVdpdGhIb2xlcyhhcnIpIHx8IF9pdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHx8IF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShhcnIsIGkpIHx8IF9ub25JdGVyYWJsZVJlc3QoKTsgfVxuXG5mdW5jdGlvbiBfbm9uSXRlcmFibGVSZXN0KCkgeyB0aHJvdyBuZXcgVHlwZUVycm9yKFwiSW52YWxpZCBhdHRlbXB0IHRvIGRlc3RydWN0dXJlIG5vbi1pdGVyYWJsZSBpbnN0YW5jZS5cXG5JbiBvcmRlciB0byBiZSBpdGVyYWJsZSwgbm9uLWFycmF5IG9iamVjdHMgbXVzdCBoYXZlIGEgW1N5bWJvbC5pdGVyYXRvcl0oKSBtZXRob2QuXCIpOyB9XG5cbmZ1bmN0aW9uIF91bnN1cHBvcnRlZEl0ZXJhYmxlVG9BcnJheShvLCBtaW5MZW4pIHsgaWYgKCFvKSByZXR1cm47IGlmICh0eXBlb2YgbyA9PT0gXCJzdHJpbmdcIikgcmV0dXJuIF9hcnJheUxpa2VUb0FycmF5KG8sIG1pbkxlbik7IHZhciBuID0gT2JqZWN0LnByb3RvdHlwZS50b1N0cmluZy5jYWxsKG8pLnNsaWNlKDgsIC0xKTsgaWYgKG4gPT09IFwiT2JqZWN0XCIgJiYgby5jb25zdHJ1Y3RvcikgbiA9IG8uY29uc3RydWN0b3IubmFtZTsgaWYgKG4gPT09IFwiTWFwXCIgfHwgbiA9PT0gXCJTZXRcIikgcmV0dXJuIEFycmF5LmZyb20obyk7IGlmIChuID09PSBcIkFyZ3VtZW50c1wiIHx8IC9eKD86VWl8SSludCg/Ojh8MTZ8MzIpKD86Q2xhbXBlZCk/QXJyYXkkLy50ZXN0KG4pKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgfVxuXG5mdW5jdGlvbiBfYXJyYXlMaWtlVG9BcnJheShhcnIsIGxlbikgeyBpZiAobGVuID09IG51bGwgfHwgbGVuID4gYXJyLmxlbmd0aCkgbGVuID0gYXJyLmxlbmd0aDsgZm9yICh2YXIgaSA9IDAsIGFycjIgPSBuZXcgQXJyYXkobGVuKTsgaSA8IGxlbjsgaSsrKSB7IGFycjJbaV0gPSBhcnJbaV07IH0gcmV0dXJuIGFycjI7IH1cblxuZnVuY3Rpb24gX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgeyBpZiAodHlwZW9mIFN5bWJvbCA9PT0gXCJ1bmRlZmluZWRcIiB8fCAhKFN5bWJvbC5pdGVyYXRvciBpbiBPYmplY3QoYXJyKSkpIHJldHVybjsgdmFyIF9hcnIgPSBbXTsgdmFyIF9uID0gdHJ1ZTsgdmFyIF9kID0gZmFsc2U7IHZhciBfZSA9IHVuZGVmaW5lZDsgdHJ5IHsgZm9yICh2YXIgX2kgPSBhcnJbU3ltYm9sLml0ZXJhdG9yXSgpLCBfczsgIShfbiA9IChfcyA9IF9pLm5leHQoKSkuZG9uZSk7IF9uID0gdHJ1ZSkgeyBfYXJyLnB1c2goX3MudmFsdWUpOyBpZiAoaSAmJiBfYXJyLmxlbmd0aCA9PT0gaSkgYnJlYWs7IH0gfSBjYXRjaCAoZXJyKSB7IF9kID0gdHJ1ZTsgX2UgPSBlcnI7IH0gZmluYWxseSB7IHRyeSB7IGlmICghX24gJiYgX2lbXCJyZXR1cm5cIl0gIT0gbnVsbCkgX2lbXCJyZXR1cm5cIl0oKTsgfSBmaW5hbGx5IHsgaWYgKF9kKSB0aHJvdyBfZTsgfSB9IHJldHVybiBfYXJyOyB9XG5cbmZ1bmN0aW9uIF9hcnJheVdpdGhIb2xlcyhhcnIpIHsgaWYgKEFycmF5LmlzQXJyYXkoYXJyKSkgcmV0dXJuIGFycjsgfVxuXG52YXIgX3JlcXVpcmUgPSByZXF1aXJlKCcuLi8uLi9FcnJvcicpLFxuICAgIElOVkFMSURfUk9XX0NPTCA9IF9yZXF1aXJlLklOVkFMSURfUk9XX0NPTCxcbiAgICBPVkVSRkxPV19ST1cgPSBfcmVxdWlyZS5PVkVSRkxPV19ST1csXG4gICAgSU5WQUxJRF9NQVRSSVggPSBfcmVxdWlyZS5JTlZBTElEX01BVFJJWDtcbi8qKlxyXG4gKiBHZXRzIHRoZSByb3cgb2YgYSBNYXRyaXggd2l0aCB2YWxpZCBpbmRleC5cclxuICogQG1lbWJlcm9mIE1hdHJpeFxyXG4gKiBAc3RhdGljXHJcbiAqIEBwYXJhbSB7TWF0cml4fSBBIC0gQW55IE1hdHJpeFxyXG4gKiBAcGFyYW0ge251bWJlcn0gaW5kZXggLSBBbnkgdmFsaWQgcm93IGluZGV4XHJcbiAqIEByZXR1cm5zIHtNYXRyaXh9IFJvdyBvZiBBXHJcbiAqL1xuXG5cbmZ1bmN0aW9uIHJvdyhBLCBpbmRleCkge1xuICBpZiAoIShBIGluc3RhbmNlb2YgdGhpcykpIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoSU5WQUxJRF9NQVRSSVgpO1xuICB9XG5cbiAgaWYgKCFOdW1iZXIuaXNJbnRlZ2VyKGluZGV4KSB8fCBpbmRleCA8IDApIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoSU5WQUxJRF9ST1dfQ09MKTtcbiAgfVxuXG4gIHZhciBfQSRzaXplID0gQS5zaXplKCksXG4gICAgICBfQSRzaXplMiA9IF9zbGljZWRUb0FycmF5KF9BJHNpemUsIDIpLFxuICAgICAgciA9IF9BJHNpemUyWzBdLFxuICAgICAgYyA9IF9BJHNpemUyWzFdO1xuXG4gIGlmIChpbmRleCA+PSByKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKE9WRVJGTE9XX1JPVyk7XG4gIH1cblxuICB2YXIgbWF0cml4ID0gQS5fbWF0cml4O1xuICByZXR1cm4gdGhpcy5nZW5lcmF0ZSgxLCBjLCBmdW5jdGlvbiAoaSwgaikge1xuICAgIHJldHVybiBtYXRyaXhbaW5kZXhdW2pdO1xuICB9KTtcbn1cblxuO1xubW9kdWxlLmV4cG9ydHMgPSByb3c7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbmZ1bmN0aW9uIF9zbGljZWRUb0FycmF5KGFyciwgaSkgeyByZXR1cm4gX2FycmF5V2l0aEhvbGVzKGFycikgfHwgX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgfHwgX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KGFyciwgaSkgfHwgX25vbkl0ZXJhYmxlUmVzdCgpOyB9XG5cbmZ1bmN0aW9uIF9ub25JdGVyYWJsZVJlc3QoKSB7IHRocm93IG5ldyBUeXBlRXJyb3IoXCJJbnZhbGlkIGF0dGVtcHQgdG8gZGVzdHJ1Y3R1cmUgbm9uLWl0ZXJhYmxlIGluc3RhbmNlLlxcbkluIG9yZGVyIHRvIGJlIGl0ZXJhYmxlLCBub24tYXJyYXkgb2JqZWN0cyBtdXN0IGhhdmUgYSBbU3ltYm9sLml0ZXJhdG9yXSgpIG1ldGhvZC5cIik7IH1cblxuZnVuY3Rpb24gX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KG8sIG1pbkxlbikgeyBpZiAoIW8pIHJldHVybjsgaWYgKHR5cGVvZiBvID09PSBcInN0cmluZ1wiKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgdmFyIG4gPSBPYmplY3QucHJvdG90eXBlLnRvU3RyaW5nLmNhbGwobykuc2xpY2UoOCwgLTEpOyBpZiAobiA9PT0gXCJPYmplY3RcIiAmJiBvLmNvbnN0cnVjdG9yKSBuID0gby5jb25zdHJ1Y3Rvci5uYW1lOyBpZiAobiA9PT0gXCJNYXBcIiB8fCBuID09PSBcIlNldFwiKSByZXR1cm4gQXJyYXkuZnJvbShvKTsgaWYgKG4gPT09IFwiQXJndW1lbnRzXCIgfHwgL14oPzpVaXxJKW50KD86OHwxNnwzMikoPzpDbGFtcGVkKT9BcnJheSQvLnRlc3QobikpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB9XG5cbmZ1bmN0aW9uIF9hcnJheUxpa2VUb0FycmF5KGFyciwgbGVuKSB7IGlmIChsZW4gPT0gbnVsbCB8fCBsZW4gPiBhcnIubGVuZ3RoKSBsZW4gPSBhcnIubGVuZ3RoOyBmb3IgKHZhciBpID0gMCwgYXJyMiA9IG5ldyBBcnJheShsZW4pOyBpIDwgbGVuOyBpKyspIHsgYXJyMltpXSA9IGFycltpXTsgfSByZXR1cm4gYXJyMjsgfVxuXG5mdW5jdGlvbiBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB7IGlmICh0eXBlb2YgU3ltYm9sID09PSBcInVuZGVmaW5lZFwiIHx8ICEoU3ltYm9sLml0ZXJhdG9yIGluIE9iamVjdChhcnIpKSkgcmV0dXJuOyB2YXIgX2FyciA9IFtdOyB2YXIgX24gPSB0cnVlOyB2YXIgX2QgPSBmYWxzZTsgdmFyIF9lID0gdW5kZWZpbmVkOyB0cnkgeyBmb3IgKHZhciBfaSA9IGFycltTeW1ib2wuaXRlcmF0b3JdKCksIF9zOyAhKF9uID0gKF9zID0gX2kubmV4dCgpKS5kb25lKTsgX24gPSB0cnVlKSB7IF9hcnIucHVzaChfcy52YWx1ZSk7IGlmIChpICYmIF9hcnIubGVuZ3RoID09PSBpKSBicmVhazsgfSB9IGNhdGNoIChlcnIpIHsgX2QgPSB0cnVlOyBfZSA9IGVycjsgfSBmaW5hbGx5IHsgdHJ5IHsgaWYgKCFfbiAmJiBfaVtcInJldHVyblwiXSAhPSBudWxsKSBfaVtcInJldHVyblwiXSgpOyB9IGZpbmFsbHkgeyBpZiAoX2QpIHRocm93IF9lOyB9IH0gcmV0dXJuIF9hcnI7IH1cblxuZnVuY3Rpb24gX2FycmF5V2l0aEhvbGVzKGFycikgeyBpZiAoQXJyYXkuaXNBcnJheShhcnIpKSByZXR1cm4gYXJyOyB9XG5cbmZ1bmN0aW9uIF90eXBlb2Yob2JqKSB7IFwiQGJhYmVsL2hlbHBlcnMgLSB0eXBlb2ZcIjsgaWYgKHR5cGVvZiBTeW1ib2wgPT09IFwiZnVuY3Rpb25cIiAmJiB0eXBlb2YgU3ltYm9sLml0ZXJhdG9yID09PSBcInN5bWJvbFwiKSB7IF90eXBlb2YgPSBmdW5jdGlvbiBfdHlwZW9mKG9iaikgeyByZXR1cm4gdHlwZW9mIG9iajsgfTsgfSBlbHNlIHsgX3R5cGVvZiA9IGZ1bmN0aW9uIF90eXBlb2Yob2JqKSB7IHJldHVybiBvYmogJiYgdHlwZW9mIFN5bWJvbCA9PT0gXCJmdW5jdGlvblwiICYmIG9iai5jb25zdHJ1Y3RvciA9PT0gU3ltYm9sICYmIG9iaiAhPT0gU3ltYm9sLnByb3RvdHlwZSA/IFwic3ltYm9sXCIgOiB0eXBlb2Ygb2JqOyB9OyB9IHJldHVybiBfdHlwZW9mKG9iaik7IH1cblxudmFyIF9yZXF1aXJlID0gcmVxdWlyZSgnLi4vLi4vRXJyb3InKSxcbiAgICBJTlZBTElEX01BVFJJWCA9IF9yZXF1aXJlLklOVkFMSURfTUFUUklYLFxuICAgIEVYUEVDVEVEX1NUUklOR19OVU1CRVJfQVRfUE9TXzFfMiA9IF9yZXF1aXJlLkVYUEVDVEVEX1NUUklOR19OVU1CRVJfQVRfUE9TXzFfMixcbiAgICBJTlZBTElEX1JPVyA9IF9yZXF1aXJlLklOVkFMSURfUk9XLFxuICAgIElOVkFMSURfQ09MVU1OID0gX3JlcXVpcmUuSU5WQUxJRF9DT0xVTU4sXG4gICAgT1ZFUkZMT1dfUk9XID0gX3JlcXVpcmUuT1ZFUkZMT1dfUk9XLFxuICAgIElOVkFMSURfUk9XU19FWFBSRVNTSU9OID0gX3JlcXVpcmUuSU5WQUxJRF9ST1dTX0VYUFJFU1NJT04sXG4gICAgSU5WQUxJRF9DT0xVTU5TX0VYUFJFU1NJT04gPSBfcmVxdWlyZS5JTlZBTElEX0NPTFVNTlNfRVhQUkVTU0lPTixcbiAgICBPVkVSRkxPV19DT0xVTU4gPSBfcmVxdWlyZS5PVkVSRkxPV19DT0xVTU47XG4vKipcclxuICogR2VuZXJhdGVzIGEgc3VibWF0cml4IG9mIGEgbWF0cml4LlxyXG4gKiBAbWVtYmVyb2YgTWF0cml4XHJcbiAqIEBzdGF0aWNcclxuICogQHBhcmFtIHtNYXRyaXh9IEEgLSBBbnkgbWF0cml4XHJcbiAqIEBwYXJhbSB7c3RyaW5nfG51bWJlcn0gcm93cyAtIFJvd3MgZXhwcmVzc2lvblxyXG4gKiBAcGFyYW0ge3N0cmluZ3xudW1iZXJ9IGNvbHMgLSBDb2x1bW5zIGV4cHJlc3Npb25cclxuICogQHJldHVybnMge01hdHJpeH0gU3VibWF0cml4IG9mIEFcclxuICovXG5cblxuZnVuY3Rpb24gc3VibWF0cml4KEEsIHJvd3MsIGNvbHMpIHtcbiAgaWYgKCEoQSBpbnN0YW5jZW9mIHRoaXMpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfTUFUUklYKTtcbiAgfVxuXG4gIHZhciBhcmcxVHlwZSA9IF90eXBlb2Yocm93cyk7XG5cbiAgdmFyIGFyZzJUeXBlID0gX3R5cGVvZihjb2xzKTtcblxuICBpZiAoYXJnMVR5cGUgIT09ICdzdHJpbmcnICYmIGFyZzFUeXBlICE9PSAnbnVtYmVyJyB8fCBhcmcyVHlwZSAhPT0gJ3N0cmluZycgJiYgYXJnMlR5cGUgIT09ICdudW1iZXInKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKEVYUEVDVEVEX1NUUklOR19OVU1CRVJfQVRfUE9TXzFfMik7XG4gIH1cblxuICB2YXIgX0Ekc2l6ZSA9IEEuc2l6ZSgpLFxuICAgICAgX0Ekc2l6ZTIgPSBfc2xpY2VkVG9BcnJheShfQSRzaXplLCAyKSxcbiAgICAgIHJvdyA9IF9BJHNpemUyWzBdLFxuICAgICAgY29sID0gX0Ekc2l6ZTJbMV07XG5cbiAgdmFyIHJvd1N0YXJ0O1xuICB2YXIgcm93RW5kO1xuICB2YXIgY29sU3RhcnQ7XG4gIHZhciBjb2xFbmQ7XG5cbiAgaWYgKGFyZzFUeXBlID09PSAnbnVtYmVyJykge1xuICAgIGlmICghTnVtYmVyLmlzSW50ZWdlcihyb3dzKSB8fCByb3dzIDwgMCkge1xuICAgICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfUk9XKTtcbiAgICB9XG5cbiAgICBpZiAocm93cyA+PSByb3cpIHtcbiAgICAgIHRocm93IG5ldyBFcnJvcihPVkVSRkxPV19ST1cpO1xuICAgIH1cblxuICAgIHJvd1N0YXJ0ID0gcm93cztcbiAgICByb3dFbmQgPSByb3dzO1xuICB9IGVsc2Uge1xuICAgIC8vIHN0cmluZ1xuICAgIHZhciBhcmcgPSByb3dzLnNwbGl0KCc6Jyk7XG5cbiAgICBpZiAoYXJnLmxlbmd0aCAhPT0gMikge1xuICAgICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfUk9XU19FWFBSRVNTSU9OKTtcbiAgICB9XG5cbiAgICB2YXIgX2FyZyA9IF9zbGljZWRUb0FycmF5KGFyZywgMiksXG4gICAgICAgIHIxID0gX2FyZ1swXSxcbiAgICAgICAgcjIgPSBfYXJnWzFdO1xuXG4gICAgaWYgKHIxID09PSAnJykge1xuICAgICAgcm93U3RhcnQgPSAwO1xuICAgIH0gZWxzZSB7XG4gICAgICB2YXIgciA9IE51bWJlcihyMSk7XG5cbiAgICAgIGlmICghTnVtYmVyLmlzSW50ZWdlcihyKSB8fCByIDwgMCkge1xuICAgICAgICB0aHJvdyBuZXcgRXJyb3IoSU5WQUxJRF9ST1cpO1xuICAgICAgfVxuXG4gICAgICBpZiAociA+PSByb3cpIHtcbiAgICAgICAgdGhyb3cgbmV3IEVycm9yKE9WRVJGTE9XX1JPVyk7XG4gICAgICB9XG5cbiAgICAgIHJvd1N0YXJ0ID0gcjtcbiAgICB9XG5cbiAgICBpZiAocjIgPT09ICcnKSB7XG4gICAgICByb3dFbmQgPSByb3cgLSAxO1xuICAgIH0gZWxzZSB7XG4gICAgICB2YXIgX3IgPSBOdW1iZXIocjIpO1xuXG4gICAgICBpZiAoIU51bWJlci5pc0ludGVnZXIoX3IpIHx8IF9yIDwgMCkge1xuICAgICAgICB0aHJvdyBuZXcgRXJyb3IoSU5WQUxJRF9ST1cpO1xuICAgICAgfVxuXG4gICAgICBpZiAoX3IgPj0gcm93KSB7XG4gICAgICAgIHRocm93IG5ldyBFcnJvcihPVkVSRkxPV19ST1cpO1xuICAgICAgfVxuXG4gICAgICByb3dFbmQgPSBfcjtcbiAgICB9XG5cbiAgICBpZiAocm93U3RhcnQgPiByb3dFbmQpIHtcbiAgICAgIHRocm93IG5ldyBFcnJvcihJTlZBTElEX1JPV1NfRVhQUkVTU0lPTik7XG4gICAgfVxuICB9XG5cbiAgaWYgKGFyZzJUeXBlID09PSAnbnVtYmVyJykge1xuICAgIGlmICghTnVtYmVyLmlzSW50ZWdlcihjb2xzKSB8fCBjb2xzIDwgMCkge1xuICAgICAgdGhyb3cgbmV3IEVycm9yKElOVkFMSURfQ09MVU1OKTtcbiAgICB9XG5cbiAgICBpZiAoY29scyA+PSBjb2wpIHtcbiAgICAgIHRocm93IG5ldyBFcnJvcihPVkVSRkxPV19DT0xVTU4pO1xuICAgIH1cblxuICAgIGNvbFN0YXJ0ID0gY29scztcbiAgICBjb2xFbmQgPSBjb2xzO1xuICB9IGVsc2Uge1xuICAgIC8vIHN0cmluZ1xuICAgIHZhciBfYXJnMiA9IGNvbHMuc3BsaXQoJzonKTtcblxuICAgIGlmIChfYXJnMi5sZW5ndGggIT09IDIpIHtcbiAgICAgIHRocm93IG5ldyBFcnJvcihJTlZBTElEX0NPTFVNTlNfRVhQUkVTU0lPTik7XG4gICAgfVxuXG4gICAgdmFyIF9hcmczID0gX3NsaWNlZFRvQXJyYXkoX2FyZzIsIDIpLFxuICAgICAgICBjMSA9IF9hcmczWzBdLFxuICAgICAgICBjMiA9IF9hcmczWzFdO1xuXG4gICAgaWYgKGMxID09PSAnJykge1xuICAgICAgY29sU3RhcnQgPSAwO1xuICAgIH0gZWxzZSB7XG4gICAgICB2YXIgYyA9IE51bWJlcihjMSk7XG5cbiAgICAgIGlmICghTnVtYmVyLmlzSW50ZWdlcihjKSB8fCBjIDwgMCkge1xuICAgICAgICB0aHJvdyBuZXcgRXJyb3IoSU5WQUxJRF9DT0xVTU4pO1xuICAgICAgfVxuXG4gICAgICBpZiAoYyA+PSBjb2wpIHtcbiAgICAgICAgdGhyb3cgbmV3IEVycm9yKE9WRVJGTE9XX0NPTFVNTik7XG4gICAgICB9XG5cbiAgICAgIGNvbFN0YXJ0ID0gYztcbiAgICB9XG5cbiAgICBpZiAoYzIgPT09ICcnKSB7XG4gICAgICBjb2xFbmQgPSBjb2wgLSAxO1xuICAgIH0gZWxzZSB7XG4gICAgICB2YXIgX2MgPSBOdW1iZXIoYzIpO1xuXG4gICAgICBpZiAoIU51bWJlci5pc0ludGVnZXIoX2MpIHx8IF9jIDwgMCkge1xuICAgICAgICB0aHJvdyBuZXcgRXJyb3IoSU5WQUxJRF9DT0xVTU4pO1xuICAgICAgfVxuXG4gICAgICBpZiAoX2MgPj0gY29sKSB7XG4gICAgICAgIHRocm93IG5ldyBFcnJvcihPVkVSRkxPV19DT0xVTU4pO1xuICAgICAgfVxuXG4gICAgICBjb2xFbmQgPSBfYztcbiAgICB9XG5cbiAgICBpZiAoY29sU3RhcnQgPiBjb2xFbmQpIHtcbiAgICAgIHRocm93IG5ldyBFcnJvcihJTlZBTElEX0NPTFVNTlNfRVhQUkVTU0lPTik7XG4gICAgfVxuICB9XG5cbiAgdmFyIG1hdHJpeCA9IEEuX21hdHJpeDtcbiAgdmFyIHN1YlJvdyA9IHJvd0VuZCAtIHJvd1N0YXJ0ICsgMTtcbiAgdmFyIHN1YkNvbCA9IGNvbEVuZCAtIGNvbFN0YXJ0ICsgMTtcbiAgdmFyIHN1Yk1hdHJpeCA9IG5ldyBBcnJheShzdWJSb3cpO1xuXG4gIGZvciAodmFyIGkgPSByb3dTdGFydDsgaSA8PSByb3dFbmQ7IGkrKykge1xuICAgIHZhciBuZXdSb3cgPSBuZXcgQXJyYXkoc3ViQ29sKTtcblxuICAgIGZvciAodmFyIGogPSBjb2xTdGFydDsgaiA8PSBjb2xFbmQ7IGorKykge1xuICAgICAgbmV3Um93W2ogLSBjb2xTdGFydF0gPSBtYXRyaXhbaV1bal07XG4gICAgfVxuXG4gICAgc3ViTWF0cml4W2kgLSByb3dTdGFydF0gPSBuZXdSb3c7XG4gIH1cblxuICByZXR1cm4gbmV3IHRoaXMoc3ViTWF0cml4KTtcbn1cblxuO1xubW9kdWxlLmV4cG9ydHMgPSBzdWJtYXRyaXg7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbmZ1bmN0aW9uIF9zbGljZWRUb0FycmF5KGFyciwgaSkgeyByZXR1cm4gX2FycmF5V2l0aEhvbGVzKGFycikgfHwgX2l0ZXJhYmxlVG9BcnJheUxpbWl0KGFyciwgaSkgfHwgX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KGFyciwgaSkgfHwgX25vbkl0ZXJhYmxlUmVzdCgpOyB9XG5cbmZ1bmN0aW9uIF9ub25JdGVyYWJsZVJlc3QoKSB7IHRocm93IG5ldyBUeXBlRXJyb3IoXCJJbnZhbGlkIGF0dGVtcHQgdG8gZGVzdHJ1Y3R1cmUgbm9uLWl0ZXJhYmxlIGluc3RhbmNlLlxcbkluIG9yZGVyIHRvIGJlIGl0ZXJhYmxlLCBub24tYXJyYXkgb2JqZWN0cyBtdXN0IGhhdmUgYSBbU3ltYm9sLml0ZXJhdG9yXSgpIG1ldGhvZC5cIik7IH1cblxuZnVuY3Rpb24gX3Vuc3VwcG9ydGVkSXRlcmFibGVUb0FycmF5KG8sIG1pbkxlbikgeyBpZiAoIW8pIHJldHVybjsgaWYgKHR5cGVvZiBvID09PSBcInN0cmluZ1wiKSByZXR1cm4gX2FycmF5TGlrZVRvQXJyYXkobywgbWluTGVuKTsgdmFyIG4gPSBPYmplY3QucHJvdG90eXBlLnRvU3RyaW5nLmNhbGwobykuc2xpY2UoOCwgLTEpOyBpZiAobiA9PT0gXCJPYmplY3RcIiAmJiBvLmNvbnN0cnVjdG9yKSBuID0gby5jb25zdHJ1Y3Rvci5uYW1lOyBpZiAobiA9PT0gXCJNYXBcIiB8fCBuID09PSBcIlNldFwiKSByZXR1cm4gQXJyYXkuZnJvbShvKTsgaWYgKG4gPT09IFwiQXJndW1lbnRzXCIgfHwgL14oPzpVaXxJKW50KD86OHwxNnwzMikoPzpDbGFtcGVkKT9BcnJheSQvLnRlc3QobikpIHJldHVybiBfYXJyYXlMaWtlVG9BcnJheShvLCBtaW5MZW4pOyB9XG5cbmZ1bmN0aW9uIF9hcnJheUxpa2VUb0FycmF5KGFyciwgbGVuKSB7IGlmIChsZW4gPT0gbnVsbCB8fCBsZW4gPiBhcnIubGVuZ3RoKSBsZW4gPSBhcnIubGVuZ3RoOyBmb3IgKHZhciBpID0gMCwgYXJyMiA9IG5ldyBBcnJheShsZW4pOyBpIDwgbGVuOyBpKyspIHsgYXJyMltpXSA9IGFycltpXTsgfSByZXR1cm4gYXJyMjsgfVxuXG5mdW5jdGlvbiBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB7IGlmICh0eXBlb2YgU3ltYm9sID09PSBcInVuZGVmaW5lZFwiIHx8ICEoU3ltYm9sLml0ZXJhdG9yIGluIE9iamVjdChhcnIpKSkgcmV0dXJuOyB2YXIgX2FyciA9IFtdOyB2YXIgX24gPSB0cnVlOyB2YXIgX2QgPSBmYWxzZTsgdmFyIF9lID0gdW5kZWZpbmVkOyB0cnkgeyBmb3IgKHZhciBfaSA9IGFycltTeW1ib2wuaXRlcmF0b3JdKCksIF9zOyAhKF9uID0gKF9zID0gX2kubmV4dCgpKS5kb25lKTsgX24gPSB0cnVlKSB7IF9hcnIucHVzaChfcy52YWx1ZSk7IGlmIChpICYmIF9hcnIubGVuZ3RoID09PSBpKSBicmVhazsgfSB9IGNhdGNoIChlcnIpIHsgX2QgPSB0cnVlOyBfZSA9IGVycjsgfSBmaW5hbGx5IHsgdHJ5IHsgaWYgKCFfbiAmJiBfaVtcInJldHVyblwiXSAhPSBudWxsKSBfaVtcInJldHVyblwiXSgpOyB9IGZpbmFsbHkgeyBpZiAoX2QpIHRocm93IF9lOyB9IH0gcmV0dXJuIF9hcnI7IH1cblxuZnVuY3Rpb24gX2FycmF5V2l0aEhvbGVzKGFycikgeyBpZiAoQXJyYXkuaXNBcnJheShhcnIpKSByZXR1cm4gYXJyOyB9XG5cbi8qKlxyXG4gKiBHZXRzIHRoZSBzdHJpbmdpZmllZCBNYXRyaXhcclxuICogQG1lbWJlcm9mIE1hdHJpeFxyXG4gKiBAaW5zdGFuY2VcclxuICogQHJldHVybnMge3N0cmluZ30gU3RyaW5naWZpZWQgTWF0cml4XHJcbiAqL1xuZnVuY3Rpb24gdG9TdHJpbmcoKSB7XG4gIHZhciBtYXRyaXggPSB0aGlzLl9tYXRyaXg7XG5cbiAgdmFyIF90aGlzJHNpemUgPSB0aGlzLnNpemUoKSxcbiAgICAgIF90aGlzJHNpemUyID0gX3NsaWNlZFRvQXJyYXkoX3RoaXMkc2l6ZSwgMiksXG4gICAgICByb3cgPSBfdGhpcyRzaXplMlswXSxcbiAgICAgIGNvbCA9IF90aGlzJHNpemUyWzFdO1xuXG4gIHZhciBzdHIgPSAnJztcblxuICBmb3IgKHZhciBpID0gMDsgaSA8IHJvdzsgaSsrKSB7XG4gICAgZm9yICh2YXIgaiA9IDA7IGogPCBjb2w7IGorKykge1xuICAgICAgc3RyICs9IG1hdHJpeFtpXVtqXS50b1N0cmluZygpO1xuXG4gICAgICBpZiAoaiAhPT0gY29sIC0gMSkge1xuICAgICAgICBzdHIgKz0gJyAnO1xuICAgICAgfVxuICAgIH1cblxuICAgIGlmIChpICE9PSByb3cgLSAxKSB7XG4gICAgICBzdHIgKz0gJ1xcbic7XG4gICAgfVxuICB9XG5cbiAgcmV0dXJuIHN0cjtcbn1cblxuO1xubW9kdWxlLmV4cG9ydHMgPSB0b1N0cmluZzsiLCJcInVzZSBzdHJpY3RcIjtcblxuLyoqXHJcbiAqIEdlbmVyYXRlcyBhIHplcm8gTWF0cml4XHJcbiAqIEBtZW1iZXJvZiBNYXRyaXhcclxuICogQHN0YXRpY1xyXG4gKiBAcGFyYW0ge251bWJlcn0gcm93IC0gTnVtYmVyIG9mIHJvd3Mgb2YgdGhlIE1hdHJpeFxyXG4gKiBAcGFyYW0ge251bWJlcn0gY29sIC0gTnVtYmVyIG9mIGNvbHVtbnMgb2YgdGhlIE1hdHJpeFxyXG4gKiBAcmV0dXJucyB7TWF0cml4fSBaZXJvIE1hdHJpeFxyXG4gKi9cbmZ1bmN0aW9uIHplcm8ocm93LCBjb2wpIHtcbiAgaWYgKGNvbCA9PT0gdW5kZWZpbmVkKSB7XG4gICAgcmV0dXJuIHRoaXMuZ2VuZXJhdGUocm93LCByb3csIGZ1bmN0aW9uICgpIHtcbiAgICAgIHJldHVybiAwO1xuICAgIH0pO1xuICB9XG5cbiAgcmV0dXJuIHRoaXMuZ2VuZXJhdGUocm93LCBjb2wsIGZ1bmN0aW9uICgpIHtcbiAgICByZXR1cm4gMDtcbiAgfSk7XG59XG5cbjtcbm1vZHVsZS5leHBvcnRzID0gemVybzsiLCJcInVzZSBzdHJpY3RcIjtcblxudmFyIGlzTWF0cml4ID0gcmVxdWlyZSgnLi91dGlsL2lzTWF0cml4Jyk7XG5cbnZhciBfcmVxdWlyZSA9IHJlcXVpcmUoJy4vRXJyb3InKSxcbiAgICBJTlZBTElEX01BVFJJWCA9IF9yZXF1aXJlLklOVkFMSURfTUFUUklYO1xuLyoqXHJcbiAqIENyZWF0ZXMgYSBuZXcgTWF0cml4XHJcbiAqIEBuYW1lc3BhY2UgTWF0cml4XHJcbiAqIEBjbGFzc1xyXG4gKiBAcGFyYW0ge251bWJlcltdW119IEEgLSBUd28gZGltZW5zaW9uYWwgYXJyYXkgd2hlcmVcclxuICogQVtpXVtqXSByZXByZXNlbnRzIHRoZSBpLXRoIHJvdyBhbmQgai10aCBjb2x1bW4gb2YgYSBtYXRyaXhcclxuICovXG5cblxuZnVuY3Rpb24gTWF0cml4KEEpIHtcbiAgaWYgKCFpc01hdHJpeChBKSkge1xuICAgIHRocm93IG5ldyBFcnJvcihJTlZBTElEX01BVFJJWCk7XG4gIH1cblxuICB0aGlzLl9tYXRyaXggPSBBO1xuICB0aGlzLl9kaWdpdCA9IDg7XG59XG5cbm1vZHVsZS5leHBvcnRzID0gTWF0cml4OyAvLyBzdHJ1Y3R1cmVcblxuTWF0cml4LnByb3RvdHlwZS5pc0RpYWdvbmFsID0gcmVxdWlyZSgnLi9jb3JlL3N0cnVjdHVyZS9pc0RpYWdvbmFsJyk7XG5NYXRyaXgucHJvdG90eXBlLmlzU2tld1N5bW1ldHJpYyA9IHJlcXVpcmUoJy4vY29yZS9zdHJ1Y3R1cmUvaXNTa2V3U3ltbWV0cmljJyk7XG5NYXRyaXgucHJvdG90eXBlLmlzU3F1YXJlID0gcmVxdWlyZSgnLi9jb3JlL3N0cnVjdHVyZS9pc1NxdWFyZScpO1xuTWF0cml4LnByb3RvdHlwZS5pc1N5bW1ldHJpYyA9IHJlcXVpcmUoJy4vY29yZS9zdHJ1Y3R1cmUvaXNTeW1tZXRyaWMnKTtcbk1hdHJpeC5wcm90b3R5cGUuaXNMb3dlclRyaWFuZ3VsYXIgPSByZXF1aXJlKCcuL2NvcmUvc3RydWN0dXJlL2lzTG93ZXJUcmlhbmd1bGFyJyk7XG5NYXRyaXgucHJvdG90eXBlLmlzVXBwZXJUcmlhbmd1bGFyID0gcmVxdWlyZSgnLi9jb3JlL3N0cnVjdHVyZS9pc1VwcGVyVHJpYW5ndWxhcicpO1xuTWF0cml4LnByb3RvdHlwZS5pc09ydGhvZ29uYWwgPSByZXF1aXJlKCcuL2NvcmUvc3RydWN0dXJlL2lzT3J0aG9nb25hbCcpOyAvLyBwcm9wZXJ0eVxuXG5NYXRyaXgucHJvdG90eXBlLmNvbmQgPSByZXF1aXJlKCcuL2NvcmUvcHJvcGVydGllcy9jb25kJyk7XG5NYXRyaXgucHJvdG90eXBlLmRldCA9IHJlcXVpcmUoJy4vY29yZS9wcm9wZXJ0aWVzL2RldCcpO1xuTWF0cml4LnByb3RvdHlwZS5laWdlbnZhbHVlcyA9IHJlcXVpcmUoJy4vY29yZS9wcm9wZXJ0aWVzL2VpZ2VudmFsdWVzJyk7XG5NYXRyaXgucHJvdG90eXBlLm51bGxpdHkgPSByZXF1aXJlKCcuL2NvcmUvcHJvcGVydGllcy9udWxsaXR5Jyk7XG5NYXRyaXgucHJvdG90eXBlLm5vcm0gPSByZXF1aXJlKCcuL2NvcmUvcHJvcGVydGllcy9ub3JtJyk7XG5NYXRyaXgucHJvdG90eXBlLnJhbmsgPSByZXF1aXJlKCcuL2NvcmUvcHJvcGVydGllcy9yYW5rJyk7XG5NYXRyaXgucHJvdG90eXBlLnNpemUgPSByZXF1aXJlKCcuL2NvcmUvcHJvcGVydGllcy9zaXplJyk7XG5NYXRyaXgucHJvdG90eXBlLnRyYWNlID0gcmVxdWlyZSgnLi9jb3JlL3Byb3BlcnRpZXMvdHJhY2UnKTsgLy8gb3BlcmF0aW9uc1xuXG5NYXRyaXguYWRkID0gcmVxdWlyZSgnLi9jb3JlL29wZXJhdGlvbnMvYWRkJyk7XG5NYXRyaXguaW52ZXJzZSA9IHJlcXVpcmUoJy4vY29yZS9vcGVyYXRpb25zL2ludmVyc2UnKTtcbk1hdHJpeC5tdWx0aXBseSA9IHJlcXVpcmUoJy4vY29yZS9vcGVyYXRpb25zL211bHRpcGx5Jyk7XG5NYXRyaXgucG93ID0gcmVxdWlyZSgnLi9jb3JlL29wZXJhdGlvbnMvcG93Jyk7XG5NYXRyaXguc3VidHJhY3QgPSByZXF1aXJlKCcuL2NvcmUvb3BlcmF0aW9ucy9zdWJ0cmFjdCcpO1xuTWF0cml4LnRyYW5zcG9zZSA9IHJlcXVpcmUoJy4vY29yZS9vcGVyYXRpb25zL3RyYW5zcG9zZScpOyAvLyBMaW5lYXItZXF1YXRpb25zXG5cbk1hdHJpeC5iYWNrd2FyZCA9IHJlcXVpcmUoJy4vY29yZS9saW5lYXItZXF1YXRpb25zL2JhY2t3YXJkJyk7XG5NYXRyaXguZm9yd2FyZCA9IHJlcXVpcmUoJy4vY29yZS9saW5lYXItZXF1YXRpb25zL2ZvcndhcmQnKTtcbk1hdHJpeC5zb2x2ZSA9IHJlcXVpcmUoJy4vY29yZS9saW5lYXItZXF1YXRpb25zL3NvbHZlJyk7IC8vIGRlY29tcG9zaXRpb25zXG5cbk1hdHJpeC5MVSA9IHJlcXVpcmUoJy4vY29yZS9kZWNvbXBvc2l0aW9ucy9MVScpO1xuTWF0cml4LlFSID0gcmVxdWlyZSgnLi9jb3JlL2RlY29tcG9zaXRpb25zL1FSJyk7IC8vIHV0aWxzXG5cbk1hdHJpeC5jbG9uZSA9IHJlcXVpcmUoJy4vY29yZS91dGlscy9jbG9uZScpO1xuTWF0cml4LmNvbHVtbiA9IHJlcXVpcmUoJy4vY29yZS91dGlscy9jb2x1bW4nKTtcbk1hdHJpeC5kaWFnID0gcmVxdWlyZSgnLi9jb3JlL3V0aWxzL2RpYWcnKTtcbk1hdHJpeC5lbGVtZW50d2lzZSA9IHJlcXVpcmUoJy4vY29yZS91dGlscy9lbGVtZW50d2lzZScpO1xuTWF0cml4LmdlbmVyYXRlID0gcmVxdWlyZSgnLi9jb3JlL3V0aWxzL2dlbmVyYXRlJyk7XG5NYXRyaXguZ2V0RGlhZyA9IHJlcXVpcmUoJy4vY29yZS91dGlscy9nZXREaWFnJyk7XG5NYXRyaXguZ2V0UmFuZG9tTWF0cml4ID0gcmVxdWlyZSgnLi9jb3JlL3V0aWxzL2dldFJhbmRvbU1hdHJpeCcpO1xuTWF0cml4LmlkZW50aXR5ID0gcmVxdWlyZSgnLi9jb3JlL3V0aWxzL2lkZW50aXR5Jyk7XG5NYXRyaXguaXNFcXVhbCA9IHJlcXVpcmUoJy4vY29yZS91dGlscy9pc0VxdWFsJyk7XG5NYXRyaXgucm93ID0gcmVxdWlyZSgnLi9jb3JlL3V0aWxzL3JvdycpO1xuTWF0cml4LnN1Ym1hdHJpeCA9IHJlcXVpcmUoJy4vY29yZS91dGlscy9zdWJtYXRyaXgnKTtcbk1hdHJpeC56ZXJvID0gcmVxdWlyZSgnLi9jb3JlL3V0aWxzL3plcm8nKTtcbk1hdHJpeC5wcm90b3R5cGUuZW50cnkgPSByZXF1aXJlKCcuL2NvcmUvdXRpbHMvZW50cnknKTtcbk1hdHJpeC5wcm90b3R5cGUudG9TdHJpbmcgPSByZXF1aXJlKCcuL2NvcmUvdXRpbHMvdG9TdHJpbmcnKTsiLCJcInVzZSBzdHJpY3RcIjtcblxudmFyIF9yZXF1aXJlID0gcmVxdWlyZSgnLi4vRXJyb3InKSxcbiAgICBJTlZBTElEX1JPV19DT0wgPSBfcmVxdWlyZS5JTlZBTElEX1JPV19DT0w7XG5cbm1vZHVsZS5leHBvcnRzID0gZnVuY3Rpb24gZW1wdHkocm93LCBjb2wpIHtcbiAgaWYgKCFOdW1iZXIuaXNJbnRlZ2VyKHJvdykgfHwgcm93IDwgMCB8fCAhTnVtYmVyLmlzSW50ZWdlcihjb2wpIHx8IGNvbCA8IDApIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoSU5WQUxJRF9ST1dfQ09MKTtcbiAgfVxuXG4gIGlmIChyb3cgPT09IDAgfHwgY29sID09PSAwKSB7XG4gICAgcmV0dXJuIFtdO1xuICB9XG5cbiAgdmFyIG1hdHJpeCA9IG5ldyBBcnJheShyb3cpO1xuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgcm93OyBpKyspIHtcbiAgICBtYXRyaXhbaV0gPSBuZXcgQXJyYXkoY29sKTtcbiAgfVxuXG4gIHJldHVybiBtYXRyaXg7XG59OyIsIlwidXNlIHN0cmljdFwiO1xuXG52YXIgaXNOdW1iZXIgPSByZXF1aXJlKCcuL2lzTnVtYmVyJyk7XG5cbm1vZHVsZS5leHBvcnRzID0gZnVuY3Rpb24gaXNNYXRyaXgobWF0cml4KSB7XG4gIGlmICghQXJyYXkuaXNBcnJheShtYXRyaXgpKSB7XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG5cbiAgdmFyIGhlaWdodCA9IG1hdHJpeC5sZW5ndGg7XG5cbiAgaWYgKGhlaWdodCA9PT0gMCkge1xuICAgIHJldHVybiB0cnVlOyAvLyBbXSByZXByZXNlbnRzIGVtcHR5IG1hdHJpeCAoMCB4IDAgbWF0cml4KVxuICB9XG5cbiAgdmFyIGZpcnN0Um93ID0gbWF0cml4WzBdO1xuXG4gIGlmICghQXJyYXkuaXNBcnJheShmaXJzdFJvdykpIHtcbiAgICByZXR1cm4gZmFsc2U7XG4gIH1cblxuICB2YXIgd2lkdGggPSBmaXJzdFJvdy5sZW5ndGg7XG5cbiAgaWYgKHdpZHRoID09PSAwKSB7XG4gICAgcmV0dXJuIGZhbHNlOyAvLyBbIFtdIF0gaXMgbm90IGFsbG93ZWRcbiAgfVxuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgaGVpZ2h0OyBpKyspIHtcbiAgICB2YXIgcm93ID0gbWF0cml4W2ldO1xuXG4gICAgaWYgKCFBcnJheS5pc0FycmF5KHJvdykgfHwgcm93Lmxlbmd0aCAhPT0gd2lkdGgpIHtcbiAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9XG5cbiAgICBmb3IgKHZhciBqID0gMDsgaiA8IHdpZHRoOyBqKyspIHtcbiAgICAgIGlmICghaXNOdW1iZXIocm93W2pdKSkge1xuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgICB9XG4gICAgfVxuICB9XG5cbiAgcmV0dXJuIHRydWU7XG59OyIsIlwidXNlIHN0cmljdFwiO1xuXG5tb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uIGlzTnVtYmVyKF9pbnQpIHtcbiAgcmV0dXJuIE51bWJlci5pc0Zpbml0ZShfaW50KTtcbn07IiwidmFyIFN5bHZlc3RlciA9IHt9XG5cblN5bHZlc3Rlci5NYXRyaXggPSBmdW5jdGlvbiAoKSB7fVxuXG5TeWx2ZXN0ZXIuTWF0cml4LmNyZWF0ZSA9IGZ1bmN0aW9uIChlbGVtZW50cykge1xuICB2YXIgTSA9IG5ldyBTeWx2ZXN0ZXIuTWF0cml4KClcbiAgcmV0dXJuIE0uc2V0RWxlbWVudHMoZWxlbWVudHMpXG59XG5cblN5bHZlc3Rlci5NYXRyaXguSSA9IGZ1bmN0aW9uIChuKSB7XG4gIHZhciBlbHMgPSBbXSxcbiAgICBpID0gbixcbiAgICBqXG4gIHdoaWxlIChpLS0pIHtcbiAgICBqID0gblxuICAgIGVsc1tpXSA9IFtdXG4gICAgd2hpbGUgKGotLSkge1xuICAgICAgZWxzW2ldW2pdID0gaSA9PT0gaiA/IDEgOiAwXG4gICAgfVxuICB9XG4gIHJldHVybiBTeWx2ZXN0ZXIuTWF0cml4LmNyZWF0ZShlbHMpXG59XG5cblN5bHZlc3Rlci5NYXRyaXgucHJvdG90eXBlID0ge1xuICBkdXA6IGZ1bmN0aW9uICgpIHtcbiAgICByZXR1cm4gU3lsdmVzdGVyLk1hdHJpeC5jcmVhdGUodGhpcy5lbGVtZW50cylcbiAgfSxcblxuICBpc1NxdWFyZTogZnVuY3Rpb24gKCkge1xuICAgIHZhciBjb2xzID0gdGhpcy5lbGVtZW50cy5sZW5ndGggPT09IDAgPyAwIDogdGhpcy5lbGVtZW50c1swXS5sZW5ndGhcbiAgICByZXR1cm4gdGhpcy5lbGVtZW50cy5sZW5ndGggPT09IGNvbHNcbiAgfSxcblxuICB0b1JpZ2h0VHJpYW5ndWxhcjogZnVuY3Rpb24gKCkge1xuICAgIGlmICh0aGlzLmVsZW1lbnRzLmxlbmd0aCA9PT0gMCkgcmV0dXJuIFN5bHZlc3Rlci5NYXRyaXguY3JlYXRlKFtdKVxuICAgIHZhciBNID0gdGhpcy5kdXAoKSxcbiAgICAgIGVsc1xuICAgIHZhciBuID0gdGhpcy5lbGVtZW50cy5sZW5ndGgsXG4gICAgICBpLFxuICAgICAgaixcbiAgICAgIG5wID0gdGhpcy5lbGVtZW50c1swXS5sZW5ndGgsXG4gICAgICBwXG4gICAgZm9yIChpID0gMDsgaSA8IG47IGkrKykge1xuICAgICAgaWYgKE0uZWxlbWVudHNbaV1baV0gPT09IDApIHtcbiAgICAgICAgZm9yIChqID0gaSArIDE7IGogPCBuOyBqKyspIHtcbiAgICAgICAgICBpZiAoTS5lbGVtZW50c1tqXVtpXSAhPT0gMCkge1xuICAgICAgICAgICAgZWxzID0gW11cbiAgICAgICAgICAgIGZvciAocCA9IDA7IHAgPCBucDsgcCsrKSB7XG4gICAgICAgICAgICAgIGVscy5wdXNoKE0uZWxlbWVudHNbaV1bcF0gKyBNLmVsZW1lbnRzW2pdW3BdKVxuICAgICAgICAgICAgfVxuICAgICAgICAgICAgTS5lbGVtZW50c1tpXSA9IGVsc1xuICAgICAgICAgICAgYnJlYWtcbiAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICAgIGlmIChNLmVsZW1lbnRzW2ldW2ldICE9PSAwKSB7XG4gICAgICAgIGZvciAoaiA9IGkgKyAxOyBqIDwgbjsgaisrKSB7XG4gICAgICAgICAgdmFyIG11bHRpcGxpZXIgPSBNLmVsZW1lbnRzW2pdW2ldIC8gTS5lbGVtZW50c1tpXVtpXVxuICAgICAgICAgIGVscyA9IFtdXG4gICAgICAgICAgZm9yIChwID0gMDsgcCA8IG5wOyBwKyspIHtcbiAgICAgICAgICAgIC8vIEVsZW1lbnRzIHdpdGggY29sdW1uIG51bWJlcnMgdXAgdG8gYW4gaW5jbHVkaW5nIHRoZSBudW1iZXIgb2YgdGhlXG4gICAgICAgICAgICAvLyByb3cgdGhhdCB3ZSdyZSBzdWJ0cmFjdGluZyBjYW4gc2FmZWx5IGJlIHNldCBzdHJhaWdodCB0byB6ZXJvLFxuICAgICAgICAgICAgLy8gc2luY2UgdGhhdCdzIHRoZSBwb2ludCBvZiB0aGlzIHJvdXRpbmUgYW5kIGl0IGF2b2lkcyBoYXZpbmcgdG9cbiAgICAgICAgICAgIC8vIGxvb3Agb3ZlciBhbmQgY29ycmVjdCByb3VuZGluZyBlcnJvcnMgbGF0ZXJcbiAgICAgICAgICAgIGVscy5wdXNoKFxuICAgICAgICAgICAgICBwIDw9IGkgPyAwIDogTS5lbGVtZW50c1tqXVtwXSAtIE0uZWxlbWVudHNbaV1bcF0gKiBtdWx0aXBsaWVyXG4gICAgICAgICAgICApXG4gICAgICAgICAgfVxuICAgICAgICAgIE0uZWxlbWVudHNbal0gPSBlbHNcbiAgICAgICAgfVxuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gTVxuICB9LFxuXG4gIGRldGVybWluYW50OiBmdW5jdGlvbiAoKSB7XG4gICAgaWYgKHRoaXMuZWxlbWVudHMubGVuZ3RoID09PSAwKSB7XG4gICAgICByZXR1cm4gMVxuICAgIH1cbiAgICBpZiAoIXRoaXMuaXNTcXVhcmUoKSkge1xuICAgICAgcmV0dXJuIG51bGxcbiAgICB9XG4gICAgdmFyIE0gPSB0aGlzLnRvUmlnaHRUcmlhbmd1bGFyKClcbiAgICB2YXIgZGV0ID0gTS5lbGVtZW50c1swXVswXSxcbiAgICAgIG4gPSBNLmVsZW1lbnRzLmxlbmd0aFxuICAgIGZvciAodmFyIGkgPSAxOyBpIDwgbjsgaSsrKSB7XG4gICAgICBkZXQgPSBkZXQgKiBNLmVsZW1lbnRzW2ldW2ldXG4gICAgfVxuICAgIHJldHVybiBkZXRcbiAgfSxcblxuICBpc1Npbmd1bGFyOiBmdW5jdGlvbiAoKSB7XG4gICAgcmV0dXJuIHRoaXMuaXNTcXVhcmUoKSAmJiB0aGlzLmRldGVybWluYW50KCkgPT09IDBcbiAgfSxcblxuICBhdWdtZW50OiBmdW5jdGlvbiAobWF0cml4KSB7XG4gICAgaWYgKHRoaXMuZWxlbWVudHMubGVuZ3RoID09PSAwKSB7XG4gICAgICByZXR1cm4gdGhpcy5kdXAoKVxuICAgIH1cbiAgICB2YXIgTSA9IG1hdHJpeC5lbGVtZW50cyB8fCBtYXRyaXhcbiAgICBpZiAodHlwZW9mIE1bMF1bMF0gPT09ICd1bmRlZmluZWQnKSB7XG4gICAgICBNID0gU3lsdmVzdGVyLk1hdHJpeC5jcmVhdGUoTSkuZWxlbWVudHNcbiAgICB9XG4gICAgdmFyIFQgPSB0aGlzLmR1cCgpLFxuICAgICAgY29scyA9IFQuZWxlbWVudHNbMF0ubGVuZ3RoXG4gICAgdmFyIGkgPSBULmVsZW1lbnRzLmxlbmd0aCxcbiAgICAgIG5qID0gTVswXS5sZW5ndGgsXG4gICAgICBqXG4gICAgaWYgKGkgIT09IE0ubGVuZ3RoKSB7XG4gICAgICByZXR1cm4gbnVsbFxuICAgIH1cbiAgICB3aGlsZSAoaS0tKSB7XG4gICAgICBqID0gbmpcbiAgICAgIHdoaWxlIChqLS0pIHtcbiAgICAgICAgVC5lbGVtZW50c1tpXVtjb2xzICsgal0gPSBNW2ldW2pdXG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiBUXG4gIH0sXG5cbiAgaW52ZXJzZTogZnVuY3Rpb24gKCkge1xuICAgIGlmICh0aGlzLmVsZW1lbnRzLmxlbmd0aCA9PT0gMCkge1xuICAgICAgcmV0dXJuIG51bGxcbiAgICB9XG4gICAgaWYgKCF0aGlzLmlzU3F1YXJlKCkgfHwgdGhpcy5pc1Npbmd1bGFyKCkpIHtcbiAgICAgIHJldHVybiBudWxsXG4gICAgfVxuICAgIHZhciBuID0gdGhpcy5lbGVtZW50cy5sZW5ndGgsXG4gICAgICBpID0gbixcbiAgICAgIGpcbiAgICB2YXIgTSA9IHRoaXMuYXVnbWVudChTeWx2ZXN0ZXIuTWF0cml4LkkobikpLnRvUmlnaHRUcmlhbmd1bGFyKClcbiAgICB2YXIgbnAgPSBNLmVsZW1lbnRzWzBdLmxlbmd0aCxcbiAgICAgIHAsXG4gICAgICBlbHMsXG4gICAgICBkaXZpc29yXG4gICAgdmFyIGludmVyc2VfZWxlbWVudHMgPSBbXSxcbiAgICAgIG5ld19lbGVtZW50XG4gICAgLy8gU3lsdmVzdGVyLk1hdHJpeCBpcyBub24tc2luZ3VsYXIgc28gdGhlcmUgd2lsbCBiZSBubyB6ZXJvcyBvbiB0aGVcbiAgICAvLyBkaWFnb25hbC4gQ3ljbGUgdGhyb3VnaCByb3dzIGZyb20gbGFzdCB0byBmaXJzdC5cbiAgICB3aGlsZSAoaS0tKSB7XG4gICAgICAvLyBGaXJzdCwgbm9ybWFsaXNlIGRpYWdvbmFsIGVsZW1lbnRzIHRvIDFcbiAgICAgIGVscyA9IFtdXG4gICAgICBpbnZlcnNlX2VsZW1lbnRzW2ldID0gW11cbiAgICAgIGRpdmlzb3IgPSBNLmVsZW1lbnRzW2ldW2ldXG4gICAgICBmb3IgKHAgPSAwOyBwIDwgbnA7IHArKykge1xuICAgICAgICBuZXdfZWxlbWVudCA9IE0uZWxlbWVudHNbaV1bcF0gLyBkaXZpc29yXG4gICAgICAgIGVscy5wdXNoKG5ld19lbGVtZW50KVxuICAgICAgICAvLyBTaHVmZmxlIG9mZiB0aGUgY3VycmVudCByb3cgb2YgdGhlIHJpZ2h0IGhhbmQgc2lkZSBpbnRvIHRoZSByZXN1bHRzXG4gICAgICAgIC8vIGFycmF5IGFzIGl0IHdpbGwgbm90IGJlIG1vZGlmaWVkIGJ5IGxhdGVyIHJ1bnMgdGhyb3VnaCB0aGlzIGxvb3BcbiAgICAgICAgaWYgKHAgPj0gbikge1xuICAgICAgICAgIGludmVyc2VfZWxlbWVudHNbaV0ucHVzaChuZXdfZWxlbWVudClcbiAgICAgICAgfVxuICAgICAgfVxuICAgICAgTS5lbGVtZW50c1tpXSA9IGVsc1xuICAgICAgLy8gVGhlbiwgc3VidHJhY3QgdGhpcyByb3cgZnJvbSB0aG9zZSBhYm92ZSBpdCB0byBnaXZlIHRoZSBpZGVudGl0eSBtYXRyaXhcbiAgICAgIC8vIG9uIHRoZSBsZWZ0IGhhbmQgc2lkZVxuICAgICAgaiA9IGlcbiAgICAgIHdoaWxlIChqLS0pIHtcbiAgICAgICAgZWxzID0gW11cbiAgICAgICAgZm9yIChwID0gMDsgcCA8IG5wOyBwKyspIHtcbiAgICAgICAgICBlbHMucHVzaChNLmVsZW1lbnRzW2pdW3BdIC0gTS5lbGVtZW50c1tpXVtwXSAqIE0uZWxlbWVudHNbal1baV0pXG4gICAgICAgIH1cbiAgICAgICAgTS5lbGVtZW50c1tqXSA9IGVsc1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gU3lsdmVzdGVyLk1hdHJpeC5jcmVhdGUoaW52ZXJzZV9lbGVtZW50cylcbiAgfSxcblxuICBzZXRFbGVtZW50czogZnVuY3Rpb24gKGVscykge1xuICAgIHZhciBpLFxuICAgICAgaixcbiAgICAgIGVsZW1lbnRzID0gZWxzLmVsZW1lbnRzIHx8IGVsc1xuICAgIGlmIChlbGVtZW50c1swXSAmJiB0eXBlb2YgZWxlbWVudHNbMF1bMF0gIT09ICd1bmRlZmluZWQnKSB7XG4gICAgICBpID0gZWxlbWVudHMubGVuZ3RoXG4gICAgICB0aGlzLmVsZW1lbnRzID0gW11cbiAgICAgIHdoaWxlIChpLS0pIHtcbiAgICAgICAgaiA9IGVsZW1lbnRzW2ldLmxlbmd0aFxuICAgICAgICB0aGlzLmVsZW1lbnRzW2ldID0gW11cbiAgICAgICAgd2hpbGUgKGotLSkge1xuICAgICAgICAgIHRoaXMuZWxlbWVudHNbaV1bal0gPSBlbGVtZW50c1tpXVtqXVxuICAgICAgICB9XG4gICAgICB9XG4gICAgICByZXR1cm4gdGhpc1xuICAgIH1cbiAgICB2YXIgbiA9IGVsZW1lbnRzLmxlbmd0aFxuICAgIHRoaXMuZWxlbWVudHMgPSBbXVxuICAgIGZvciAoaSA9IDA7IGkgPCBuOyBpKyspIHtcbiAgICAgIHRoaXMuZWxlbWVudHMucHVzaChbZWxlbWVudHNbaV1dKVxuICAgIH1cbiAgICByZXR1cm4gdGhpc1xuICB9LFxufVxuXG5tb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uIChlbGVtZW50cykge1xuICBjb25zdCBtYXQgPSBTeWx2ZXN0ZXIuTWF0cml4LmNyZWF0ZShlbGVtZW50cykuaW52ZXJzZSgpXG4gIGlmIChtYXQgIT09IG51bGwpIHtcbiAgICByZXR1cm4gbWF0LmVsZW1lbnRzXG4gIH0gZWxzZSB7XG4gICAgcmV0dXJuIG51bGxcbiAgfVxufVxuIiwiLy8gVGhlIG1vZHVsZSBjYWNoZVxudmFyIF9fd2VicGFja19tb2R1bGVfY2FjaGVfXyA9IHt9O1xuXG4vLyBUaGUgcmVxdWlyZSBmdW5jdGlvblxuZnVuY3Rpb24gX193ZWJwYWNrX3JlcXVpcmVfXyhtb2R1bGVJZCkge1xuXHQvLyBDaGVjayBpZiBtb2R1bGUgaXMgaW4gY2FjaGVcblx0aWYoX193ZWJwYWNrX21vZHVsZV9jYWNoZV9fW21vZHVsZUlkXSkge1xuXHRcdHJldHVybiBfX3dlYnBhY2tfbW9kdWxlX2NhY2hlX19bbW9kdWxlSWRdLmV4cG9ydHM7XG5cdH1cblx0Ly8gQ3JlYXRlIGEgbmV3IG1vZHVsZSAoYW5kIHB1dCBpdCBpbnRvIHRoZSBjYWNoZSlcblx0dmFyIG1vZHVsZSA9IF9fd2VicGFja19tb2R1bGVfY2FjaGVfX1ttb2R1bGVJZF0gPSB7XG5cdFx0Ly8gbm8gbW9kdWxlLmlkIG5lZWRlZFxuXHRcdC8vIG5vIG1vZHVsZS5sb2FkZWQgbmVlZGVkXG5cdFx0ZXhwb3J0czoge31cblx0fTtcblxuXHQvLyBFeGVjdXRlIHRoZSBtb2R1bGUgZnVuY3Rpb25cblx0X193ZWJwYWNrX21vZHVsZXNfX1ttb2R1bGVJZF0obW9kdWxlLCBtb2R1bGUuZXhwb3J0cywgX193ZWJwYWNrX3JlcXVpcmVfXyk7XG5cblx0Ly8gUmV0dXJuIHRoZSBleHBvcnRzIG9mIHRoZSBtb2R1bGVcblx0cmV0dXJuIG1vZHVsZS5leHBvcnRzO1xufVxuXG4iLCIvLyBtb2R1bGUgZXhwb3J0cyBtdXN0IGJlIHJldHVybmVkIGZyb20gcnVudGltZSBzbyBlbnRyeSBpbmxpbmluZyBpcyBkaXNhYmxlZFxuLy8gc3RhcnR1cFxuLy8gTG9hZCBlbnRyeSBtb2R1bGUgYW5kIHJldHVybiBleHBvcnRzXG5yZXR1cm4gX193ZWJwYWNrX3JlcXVpcmVfXyhcIi4vaW5kZXguanNcIik7XG4iXSwic291cmNlUm9vdCI6IiJ9