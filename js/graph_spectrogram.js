"use strict";

function FlightLogSpectrogram(flightLog, canvas, analyserCanvas) {
	var canvasCtx = canvas.getContext("2d");

	var // inefficient; copied from grapher.js

		DEFAULT_FONT_FACE = "Verdana, Arial, sans-serif",

		drawingParams = {
			fontSizeFrameLabel: null,
			fontSizeFrameLabelFullscreen: "9"
		};

	var that = this;

	var mouseFrequency = null;
	var analyserZoomX = 1.0; /* 100% */
	var analyserZoomY = 1.0; /* 100% */

	var MAX_ANALYSER_LENGTH = 300 * 1000 * 1000; // 5min
	var analyserTimeRange = {
		in: 0,
		out: MAX_ANALYSER_LENGTH
	};
	var dataReload = false;

	this.setInTime = function (time) {
		analyserTimeRange.in = time;
		dataReload = true;
		return analyserTimeRange.in;
	};

	this.setOutTime = function (time) {
		if ((time - analyserTimeRange.in) <= MAX_ANALYSER_LENGTH) {
			analyserTimeRange.out = time;
			dataReload = true;
			return analyserTimeRange.out;
		}
		analyserTimeRange.out = analyserTimeRange.in + MAX_ANALYSER_LENGTH; // 5min
		dataReload = true;
		return analyserTimeRange.out;
	};

	function clampByte(v) {
		return v < 0 ? 0 : (v > 255 ? 255 : v);
	}

	function getColorMap() {
		var map = [];
		for (let i = 0; i < 256; i++) {
			let rgb = [];
			rgb[ 0 ] = clampByte(2 * (i - 64));
			rgb[ 1 ] = clampByte(4 * (i - 191));
			rgb[ 2 ] = clampByte(8 * (i - 223));
			rgb[ 3 ] = clampByte(2 * (i - 64));

			map[ i ] = rgb;
		}
		return map;
	}

	let colorMap = getColorMap();

	try {
		var sysConfig = flightLog.getSysConfig();
		var gyroRate = (1000000 / sysConfig[ 'looptime' ]).toFixed(0);
		var pidRate = 1000; //default for old logs

		var analyserZoomXElem = $("#analyserZoomX");
		var analyserZoomYElem = $("#analyserZoomY");

		// Correct the PID rate if we know the pid_process_denom (from log header)
		if (sysConfig.pid_process_denom != null) {
			pidRate = gyroRate / sysConfig.pid_process_denom;
		}

		var blackBoxRate = gyroRate * sysConfig[ 'frameIntervalPNum' ] / sysConfig[ 'frameIntervalPDenom' ];
		if (sysConfig.pid_process_denom != null) {
			blackBoxRate = blackBoxRate / sysConfig.pid_process_denom;
		}

		var dataBuffer = {
			fieldIndex: 0,
			curve: 0,
			fieldName: null
		};
		var fftData = {
			fieldIndex: -1,
			fftLength: 0,
			time: [],
			fftOutput: [],
			maxNoiseIdx: []
		};
		var windowLength = 128;
		var hanning = [];

		// Pre-calculate hanning window function
		for (var i = 0; i < windowLength; i++) {
			hanning[ i ] = 0.5 * (1 - Math.cos((2 * Math.PI * i) / (windowLength - 1)));
		}

		this.resize = function () {
			// place the sliders.
			$("input:first-of-type", canvas).css({
				left: (canvasCtx.canvas.width - 130) + "px"
			});
		};

		var dataLoad = function (fieldIndex, fieldCurve) {
			//load all samples
			var logStart = flightLog.getMinTime();
			var logEnd = flightLog.getMaxTime(); // ((flightLog.getMaxTime() - logStart) <= MAX_ANALYSER_LENGTH) ? flightLog.getMaxTime() : (logStart + MAX_ANALYSER_LENGTH);
			if (analyserTimeRange.in) {
				logStart = analyserTimeRange.in;
			}
			if (analyserTimeRange.out) {
				logEnd = analyserTimeRange.out;
			}

			var allChunks = flightLog.getChunksInTimeRange(logStart, logEnd); //Max 300 seconds

			var grandTotalValue = 0;
			var grandTotalSamples = 0;
			var parts = [];
			var c = 0;
			var p = 0;
			var cPart = [];
			var times = [];
			var partLength = windowLength / 2;

			times[ 0 ] = allChunks[ 0 ].frames[ 0 ][ FlightLogParser.prototype.FLIGHT_LOG_FIELD_INDEX_TIME ];

			// Split data into parts half window long
			for (var chunkIndex = 0; chunkIndex < allChunks.length; chunkIndex++) {
				var chunk = allChunks[ chunkIndex ];
				for (var frameIndex = 0; frameIndex < chunk.frames.length; frameIndex++) {
					cPart[ p++ ] = fieldCurve.lookupRaw(chunk.frames[ frameIndex ][ fieldIndex ]);
					if (p == partLength) {
						parts[ c++ ] = cPart;

						times[ c ] = chunk.frames[ frameIndex ][ FlightLogParser.prototype.FLIGHT_LOG_FIELD_INDEX_TIME ];
						cPart = [];
						p = 0;
					}
				}
			}

			// Set up fft
			var fftLength = windowLength;
			var fftOutput = new Float64Array(fftLength * 2);
			var fft = new FFT.complex(fftLength, false);

			// Combind two parts and apply hanning window
			for (let partIndex = 0; partIndex < parts.length - 1; partIndex++) {
				let currentPart = parts[ partIndex ];
				let nextPart = parts[ partIndex + 1 ];
				var samples = new Float64Array(windowLength);

				// Loop through all the samples in the chunks and assign them to a sample array ready to pass to the FFT.
				fftData.samples = 0;
				for (var frameIndex = 0; frameIndex < currentPart.length; frameIndex++) {
					samples[ fftData.samples++ ] = currentPart[ frameIndex ] * hanning[ frameIndex ];
				}
				for (var frameIndex = 0; frameIndex < nextPart.length; frameIndex++) {
					samples[ fftData.samples++ ] = nextPart[ frameIndex ] * hanning[ frameIndex + partLength ];
				}

				//calculate fft

				fft.simple(fftOutput, samples, 'real');

				//calculate absolute values and find motor noise above 100hz
				var maxFrequency = (blackBoxRate / 2.0);
				var noiseLowEndIdx = 100 / maxFrequency * fftLength;
				var maxNoiseIdx = 0;
				var maxNoise = 0;

				fftData.fftOutput[ partIndex ] = [];

				// Skip DC and first mode effect
				for (var i = 2; i < fftLength; i++) {
					var val = Math.abs(fftOutput[ i ]);
					grandTotalValue += val;
					grandTotalSamples++;
					fftData.fftOutput[ partIndex ][ i - 2 ] = val;
					if (i > noiseLowEndIdx && val > maxNoise) {
						maxNoise = val;
						maxNoiseIdx = i;
					}
				}

				fftData.time[ partIndex ] = times[ partIndex ];

				fftData.maxNoiseIdx[partIndex] = maxNoiseIdx / fftLength * maxFrequency;
			}

			fftData.fftLength = fftLength - 2;
			fftData.fieldIndex = fieldIndex;
			fftData.normalizingFactor = grandTotalValue/grandTotalSamples;
		};

		
		/* This function is called from the canvas drawing routines within grapher.js
		   It draws a timespan of the pre-anayzed field on screen
		   */
		this.draw = function (startTime, endTime) {
			if (!fftData.fieldIndex) return;
			// canvasCtx.lineWidth = 1;
			// canvasCtx.clearRect(0, 0, canvasCtx.canvas.width, canvasCtx.canvas.height);


			var MARGIN = 10; // pixels
			var HEIGHT = canvasCtx.canvas.height - MARGIN;
			var WIDTH = canvasCtx.canvas.width;
			var LEFT = canvasCtx.canvas.left;
			var TOP = canvasCtx.canvas.top;

			// First and last line to draw
			let startIndex = 0;
			let endIndex = 0;
			for (let index = 0; index < fftData.time.length; index++) {
				if (fftData.time[ index ] >= startTime) {
					startIndex = index;
					break;
				}
			}
			for (let index = startIndex; index < fftData.time.length; index++) {
				if (fftData.time[ index ] >= endTime) {
					endIndex = index;
					break;
				}
			}

			// Plot off screen image

			var h = fftData.fftLength;
			var w = endIndex - startIndex;
			var imageData = canvasCtx.createImageData(w, h);

			for (var imx = 0; imx < w; imx++) {
				for (var imy = 0; imy < h; imy++) {
					var raw = fftData.fftOutput[ startIndex + imx ][ h - imy - 1 ];
					var val = Math.ceil(Math.max(0, Math.min(0.3*imy/h*raw / fftData.normalizingFactor * 255, 255)));
					var rootPx = 4 * (w * imy + imx);
					var rgb = colorMap[ val ];
					imageData.data[ rootPx     ] = rgb[ 0 ];  // red color
					imageData.data[ rootPx + 1 ] = rgb[ 1 ];  // green color
					imageData.data[ rootPx + 2 ] = rgb[ 2 ];  // blue color
					imageData.data[ rootPx + 3 ] = rgb[ 3 ];  // Alpha
				}
			}

			// var PLOTTED_BUFFER_LENGTH = fftData.fftLength / (analyserZoomX);
			// var PLOTTED_BLACKBOX_RATE = blackBoxRate / (analyserZoomX);

			var newCanvas = $("<canvas>")
				.attr("width", imageData.width)
				.attr("height", imageData.height)[ 0 ];

			newCanvas.getContext("2d").putImageData(imageData, 0, 0);

			canvasCtx.save();

			//  canvasCtx.translate(0,0);//LEFT, TOP);
			canvasCtx.scale(WIDTH / w, HEIGHT / h);
			canvasCtx.drawImage(newCanvas, 0, 0);
			// 

			canvasCtx.restore();
			// var barWidth = (WIDTH / (PLOTTED_BUFFER_LENGTH / 10)) - 1;
			// var barHeight;
			// var x = 0;

			// var barGradient = canvasCtx.createLinearGradient(0,HEIGHT,0,0);
			//     barGradient.addColorStop(constrain(0/analyserZoomY,0,1),      'rgba(0,255,0,0.2)');
			//     barGradient.addColorStop(constrain(0.15/analyserZoomY,0,1),   'rgba(128,255,0,0.2)');
			//     barGradient.addColorStop(constrain(0.45/analyserZoomY,0,1),   'rgba(255,0,0,0.5)');
			//     barGradient.addColorStop(constrain(1/analyserZoomY, 0, 1),    'rgba(255,128,128,1.0)');
			// canvasCtx.fillStyle = barGradient; //'rgba(0,255,0,0.3)'; //green

			// var fftScale = HEIGHT / (analyserZoomY*100);
			// for(var i = 0; i < PLOTTED_BUFFER_LENGTH; i += 10) {
			// 	barHeight = (fftData.fftOutput[i] * fftScale);
			// 	canvasCtx.fillRect(x,(HEIGHT-barHeight),barWidth,barHeight);
			// 	x += barWidth + 1;
			// }

			// drawAxisLabel(dataBuffer.fieldName, WIDTH - 4, HEIGHT - 6, 'right');
			// drawGridLines(PLOTTED_BLACKBOX_RATE, LEFT, TOP, WIDTH, HEIGHT, MARGIN);

			var offset = 0;
			 if (mouseFrequency !=null) drawMarkerLine(mouseFrequency, blackBoxRate, '', WIDTH, HEIGHT, (15*offset++) + MARGIN, "rgba(0,255,0,0.50)", 3);
			// offset++; // make some space!
			// if ((flightLog.getSysConfig().gyro_lowpass_hz != null) && (flightLog.getSysConfig().gyro_lowpass_hz > 0)) {
			//     drawMarkerLine(flightLog.getSysConfig().gyro_lowpass_hz,  PLOTTED_BLACKBOX_RATE, 'GYRO LPF cutoff', WIDTH, HEIGHT, (15*offset++) + MARGIN, "rgba(128,255,128,0.50)");
			// }
			// if ((flightLog.getSysConfig().gyro_lowpass2_hz != null) && (flightLog.getSysConfig().gyro_lowpass2_hz > 0)) {
			//     drawMarkerLine(flightLog.getSysConfig().gyro_lowpass2_hz, PLOTTED_BLACKBOX_RATE, 'GYRO LPF2 cutoff', WIDTH, HEIGHT, (15*offset++) + MARGIN, "rgba(95,199,95,0.50)");
			// }
			// if(flightLog.getSysConfig().gyro_notch_hz!=null && flightLog.getSysConfig().gyro_notch_cutoff!=null ) {
			// 	if(flightLog.getSysConfig().gyro_notch_hz.length > 0) { //there are multiple gyro notch filters
			// 		var gradient = canvasCtx.createLinearGradient(0,0,0,(HEIGHT));
			// 		gradient.addColorStop(1,   'rgba(128,255,128,0.10)');
			// 		gradient.addColorStop(0,   'rgba(128,255,128,0.35)');
			// 		for(var i=0; i<flightLog.getSysConfig().gyro_notch_hz.length; i++) {
			// 			if(flightLog.getSysConfig().gyro_notch_hz[i] > 0 && flightLog.getSysConfig().gyro_notch_cutoff[i] > 0) {
			// 				drawMarkerLine(flightLog.getSysConfig().gyro_notch_hz[i],  PLOTTED_BLACKBOX_RATE, null, WIDTH, HEIGHT, (15*offset) + MARGIN, gradient, (flightLog.getSysConfig().gyro_notch_hz[i] - flightLog.getSysConfig().gyro_notch_cutoff[i]));
			// 				drawMarkerLine(flightLog.getSysConfig().gyro_notch_hz[i],  PLOTTED_BLACKBOX_RATE, 'GYRO notch center', WIDTH, HEIGHT, (15*offset++) + MARGIN, "rgba(128,255,128,0.50)"); // highlight the center
			// 				drawMarkerLine(flightLog.getSysConfig().gyro_notch_cutoff[i],  PLOTTED_BLACKBOX_RATE, 'GYRO notch cutoff', WIDTH, HEIGHT, (15*offset++) + MARGIN, "rgba(128,255,128,0.50)");
			// 			}
			// 		}
			// 	} else { // only a single gyro notch to display
			// 		if(flightLog.getSysConfig().gyro_notch_hz > 0 && flightLog.getSysConfig().gyro_notch_cutoff > 0) {
			// 			var gradient = canvasCtx.createLinearGradient(0,0,0,(HEIGHT));
			// 			gradient.addColorStop(1,   'rgba(128,255,128,0.10)');
			// 			gradient.addColorStop(0,   'rgba(128,255,128,0.35)');
			// 			drawMarkerLine(flightLog.getSysConfig().gyro_notch_hz,  PLOTTED_BLACKBOX_RATE, null, WIDTH, HEIGHT, (15*offset) + MARGIN, gradient, (flightLog.getSysConfig().gyro_notch_hz - flightLog.getSysConfig().gyro_notch_cutoff));
			// 			drawMarkerLine(flightLog.getSysConfig().gyro_notch_hz,  PLOTTED_BLACKBOX_RATE, 'GYRO notch center', WIDTH, HEIGHT, (15*offset++) + MARGIN, "rgba(128,255,128,0.50)"); // highlight the center
			// 			drawMarkerLine(flightLog.getSysConfig().gyro_notch_cutoff,  PLOTTED_BLACKBOX_RATE, 'GYRO notch cutoff', WIDTH, HEIGHT, (15*offset++) + MARGIN, "rgba(128,255,128,0.50)");
			// 		}
			// 	}
			// }
			// offset++; // make some space!
			// try {
			// 	if(dataBuffer.fieldName.match(/(.*yaw.*)/i)!=null) {
			// 		if(flightLog.getSysConfig().yaw_lpf_hz!=null)      		drawMarkerLine(flightLog.getSysConfig().yaw_lpf_hz,  PLOTTED_BLACKBOX_RATE, 'YAW LPF cutoff', WIDTH, HEIGHT, (15*offset++) + MARGIN);
			// 	} else {
			//         if((flightLog.getSysConfig().dterm_lpf_hz != null) && (flightLog.getSysConfig().dterm_lpf_hz > 0)) {
			//             drawMarkerLine(flightLog.getSysConfig().dterm_lpf_hz,  PLOTTED_BLACKBOX_RATE, 'D-TERM LPF cutoff', WIDTH, HEIGHT, (15*offset++) + MARGIN, "rgba(128,128,255,0.50)");
			//         }
			//         if((flightLog.getSysConfig().dterm_lpf2_hz != null) && (flightLog.getSysConfig().dterm_lpf2_hz > 0)) {
			//             drawMarkerLine(flightLog.getSysConfig().dterm_lpf2_hz,  PLOTTED_BLACKBOX_RATE, 'D-TERM LPF2 cutoff', WIDTH, HEIGHT, (15*offset++) + MARGIN, "rgba(90,90,255,0.50)");
			//         }

			// 		if(flightLog.getSysConfig().dterm_notch_hz!=null && flightLog.getSysConfig().dterm_notch_cutoff!=null ) {
			// 			if(flightLog.getSysConfig().dterm_notch_hz > 0 && flightLog.getSysConfig().dterm_notch_cutoff > 0) {
			// 				var gradient = canvasCtx.createLinearGradient(0,0,0,(HEIGHT));
			// 				gradient.addColorStop(1,   'rgba(128,128,255,0.10)');
			// 				gradient.addColorStop(0,   'rgba(128,128,255,0.35)');
			// 				drawMarkerLine(flightLog.getSysConfig().dterm_notch_hz,  PLOTTED_BLACKBOX_RATE, null, WIDTH, HEIGHT, (15*offset) + MARGIN, gradient, (flightLog.getSysConfig().dterm_notch_hz - flightLog.getSysConfig().dterm_notch_cutoff));
			// 				drawMarkerLine(flightLog.getSysConfig().dterm_notch_hz,  PLOTTED_BLACKBOX_RATE, 'D-TERM notch center', WIDTH, HEIGHT, (15*offset++) + MARGIN); // highlight the center
			// 				drawMarkerLine(flightLog.getSysConfig().dterm_notch_cutoff,  PLOTTED_BLACKBOX_RATE, 'D-TERM notch cutoff', WIDTH, HEIGHT, (15*offset++) + MARGIN);
			// 			}
			// 		}
			// 	}
			// 	offset++; // make some space!
			// } catch (e) {
			// 	console.log('Notch filter fieldName missing');
			// }
			// drawMarkerLine(fftData.maxNoiseIdx,  PLOTTED_BLACKBOX_RATE, 'Max motor noise', WIDTH, HEIGHT, (15*offset) + MARGIN, "rgba(255,0,0,0.50)", 3);

		};

		var drawMarkerLine = function (frequency, sampleRate, label, WIDTH, HEIGHT, OFFSET, stroke, lineWidth) {
			var y = HEIGHT * (1-( frequency / (sampleRate / 2))); // percentage of range where frequncy lies

			lineWidth = (lineWidth || 1);
			if (lineWidth > 5) { // is the linewidth specified as a frequency band
				lineWidth = WIDTH * (2 * lineWidth) / (sampleRate / 2);
			}
			if (lineWidth < 1) lineWidth = 1;

			canvasCtx.beginPath();
			canvasCtx.lineWidth = lineWidth || 1;
			canvasCtx.strokeStyle = stroke || "rgba(128,128,255,0.50)";

			canvasCtx.moveTo(0, y);
			canvasCtx.lineTo(WIDTH, y);

			canvasCtx.stroke();

			if (label != null) drawAxisLabel(label + ' ' + (frequency.toFixed(0)) + "Hz",OFFSET, y+2, 'left');

		};

		var drawGridLines = function (sampleRate, LEFT, TOP, WIDTH, HEIGHT, MARGIN) {

			var ticks = 5;
			var frequencyInterval = (sampleRate / ticks) / 2;
			var frequency = 0;

			for (var i = 0; i <= ticks; i++) {
				canvasCtx.beginPath();
				canvasCtx.lineWidth = 1;
				canvasCtx.strokeStyle = "rgba(255,255,255,0.25)";

				canvasCtx.moveTo(i * (WIDTH / ticks), 0);
				canvasCtx.lineTo(i * (WIDTH / ticks), HEIGHT);

				canvasCtx.stroke();
				var textAlign = (i == 0) ? 'left' : ((i == ticks) ? 'right' : 'center');
				drawAxisLabel((frequency.toFixed(0)) + "Hz", i * (WIDTH / ticks), HEIGHT + MARGIN, textAlign);
				frequency += frequencyInterval;
			}
		};

		var drawAxisLabel = function (axisLabel, X, Y, align) {
			canvasCtx.font = drawingParams.fontSizeFrameLabelFullscreen + "pt " + DEFAULT_FONT_FACE;
			canvasCtx.fillStyle = "rgba(255,255,255,0.9)";
			if (align) {
				canvasCtx.textAlign = align;
			} else {
				canvasCtx.textAlign = 'center';
			}


			canvasCtx.fillText(axisLabel, X, Y);
		};

		this.setField = function(fieldIndex, curve, fieldName)
		{
			// Detect change of selected field.... reload and redraw required.
			if ((fieldIndex != fftData.fieldIndex) || dataReload) {
				dataReload = false;
				dataLoad(fieldIndex, curve);
			}
		}

		this.destroy = function () {
			$(analyserCanvas).off("mousemove", trackFrequency);
		};

		this.refresh = function () {
			//draw();
		};

		/* Add mouse over event to read the frequency */
		$(canvas).on('mousemove', function (e) {
			trackFrequency(e, that);
		});

		/* add zoom controls */
		analyserZoomXElem.on('input',
			function () {
				analyserZoomX = (analyserZoomXElem.val() / 100);
				that.refresh();
			}
		); analyserZoomXElem.val(100);

	} catch (e) {
		console.log('Failed to create analyser... error:' + e);
	}
	// track frequency under mouse
	var lastFrequency;
	function trackFrequency(e, analyser) {
		if (e.shiftKey) {
			mouseFrequency = screenToFrequency(e.clientY);

			if (lastFrequency != mouseFrequency) {
				lastFrequency = mouseFrequency;
				//if (analyser) analyser.refresh();
			}
			e.preventDefault();
		}
	}

	function screenToFrequency(y) {
		var rect = canvas.getBoundingClientRect();
		return ((rect.bottom - y) / rect.height) * (blackBoxRate / 2);
	}

}

