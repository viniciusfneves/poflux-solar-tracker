import { ws } from "../../scripts/websocket.js";

ws.onmessage = function (response) {
	let json = JSON.parse(response.data);

	document.getElementById("lens_angle").innerHTML = json["MPU"]["lensAngle"].toFixed(1);
	document.getElementById("sun_position").innerHTML = json["sunPosition"];
	
    document.getElementById("rtc_day").innerHTML = json["RTC"]["day"];
	document.getElementById("rtc_month").innerHTML = json["RTC"]["month"];
	document.getElementById("rtc_year").innerHTML = json["RTC"]["year"];
	document.getElementById("rtc_hour").innerHTML = json["RTC"]["hour"];
	document.getElementById("rtc_minute").innerHTML = json["RTC"]["minute"];
	document.getElementById("rtc_second").innerHTML = json["RTC"]["second"];

	document.getElementById("motor_pwm").innerHTML = json["motor"]["pwm"];
	document.getElementById("motor_direction").innerHTML = json["motor"]["direction"];

	document.getElementById("kp").innerHTML = json["PID_values"]["kp"];
	document.getElementById("ki").innerHTML = json["PID_values"]["ki"];
	document.getElementById("kd").innerHTML = json["PID_values"]["kd"];
	document.getElementById("P").innerHTML = json["PID_values"]["p"].toFixed(1);
	document.getElementById("I").innerHTML = json["PID_values"]["i"].toFixed(2);
	document.getElementById("D").innerHTML = json["PID_values"]["d"].toFixed(2);
	document.getElementById("pid_output").innerHTML = json["PID_values"]["output"];
};
