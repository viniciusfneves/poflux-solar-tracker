import { ws } from "./websocket.js";

var modified = false;
var ticks = 0;

window.onload = function () {
	document
		.getElementById("auto-btn")
		.addEventListener("click", (_) => ws.send("{'mode':'auto'}"));
	document
		.getElementById("manual-btn")
		.addEventListener("click", (_) => ws.send("{'mode':'manual'}"));
	document
		.getElementById("halt-btn")
		.addEventListener("click", (_) => ws.send("{'mode':'halt'}"));
	document.getElementById("adjust_rtc").oninput = function () {
		modified = true;
		ticks = 0;
	};
	document.getElementById("rtc-set-btn").addEventListener("click", (_) => {
		let value = document.getElementById("adjust_rtc").value;
		let dateObj = new Date(value);
		let date = dateObj.getTime();
		console.log(`{'adjust':{'rtc':${date / 1000}}}`);
		//ws.send(`{'adjust':{'rtc':${date/1000}}}`);
	});
	document
		.getElementById("download-tracking-file")
		.addEventListener("click", (_) => (window.location.href = "pof-lux/tracking"));
	document
		.getElementById("clear-tracking-file")
		.addEventListener("click", (_) => (window.location.href = "pof-lux/clear_tracking"));
	// document.getElementById("debug-send-button").addEventListener("click", (_) => sendCustomMessage());

	setInterval(() => {
		if (!modified) {
			var now = new Date();
			now = now.toISOString();
			var nowStr = now.substring(0, now.lastIndexOf("."));
			document.getElementById("adjust_rtc").value = nowStr;
		} else {
			ticks++;
			if (ticks > 15) {
				modified = false;
				ticks = 0;
			}
		}
	}, 1000);
};

function setOpMode(mode) {
	var buttons = document.getElementsByClassName("mode-btn");
	for (var i = 0, len = buttons.length; i < len; i++) {
		buttons[i].style.backgroundColor = "gray";
	}
	if (mode + "-btn" == "halt-btn") {
		document.getElementById(mode + "-btn").style.backgroundColor = "#dc3545";
	} else {
		document.getElementById(mode + "-btn").style.backgroundColor = "#198754";
	}
}

ws.onmessage = function (response) {
	let json = JSON.parse(response.data);

	let now = new Date(json["RTC"] * 1000);

	setOpMode(json["mode"]);
	document.getElementById("sun_position").innerHTML = json["sunPosition"];
	document.getElementById("manual_position").innerHTML = json["manualSetpoint"];
	document.getElementById("lens_angle").innerHTML = json["MPU"]["lensAngle"].toFixed(1);

	document.getElementById("rtc_day").innerHTML = json["RTC"];
	// document.getElementById("rtc_month").innerHTML = now.getUTCMonth();
	document.getElementById("rtc_year").innerHTML = now.getFullYear();
	// document.getElementById("rtc_hour").innerHTML = now.getHours();
	// document.getElementById("rtc_minute").innerHTML = now.getMinutes();
	// document.getElementById("rtc_second").innerHTML = now.getSeconds();

	let motor_percentage = (json["motor"] / 2.55).toFixed(1);
	document.getElementById("motor-pwr-bar").innerHTML = motor_percentage + "%";
	document.getElementById("motor-pwr-bar").style.width = motor_percentage + "%";
	//document.getElementById("motor_direction").innerHTML = json["motor"]["direction"];

	document.getElementById("kp").innerHTML = json["PID_values"]["kp"];
	document.getElementById("ki").innerHTML = json["PID_values"]["ki"];
	document.getElementById("kd").innerHTML = json["PID_values"]["kd"];
	document.getElementById("P").innerHTML = json["PID_values"]["p"].toFixed(1);
	document.getElementById("I").innerHTML = json["PID_values"]["i"].toFixed(2);
	document.getElementById("D").innerHTML = json["PID_values"]["d"].toFixed(2);
	document.getElementById("pid_output").innerHTML = json["PID_values"]["output"];
};

function sendCustomMessage() {
	let message = document.getElementById("debug-message-text-field").value;
	ws.send(message);
}
