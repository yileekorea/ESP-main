var statusupdate = false;
var selected_network_ssid = "";
var lastmode = "";
var ipaddress = "";


// get statup status and populate input fields
var r1 = new XMLHttpRequest();
r1.open("GET", "status", true);
r1.onreadystatechange = function () {
  if (r1.readyState != 4 || r1.status != 200) return;

  var status_fix = r1.responseText.replace(/( �)|( �)/gi, "");
  var status = JSON.parse(status_fix);

  if  (status.pass==true){
   document.getElementById("passkey").value = status.pass;
 } else {
   document.getElementById("passkey").innerHTML = '';
 }

  if ((status.www_user!=0) && (status.www_user!="undefined")  ){
    document.getElementById("www_user").value = status.www_username;
    document.getElementById("www_pass").value = status.www_password;
  }

  if (status.mqtt_server!=0){
    document.getElementById("mqtt_server").value = status.mqtt_server;
    if (status.mqtt_user!=0){
      document.getElementById("mqtt_user").value = status.mqtt_user;
      document.getElementById("mqtt_pass").value = status.mqtt_pass;
    }
  }

  if (status.mqtt_connected == "1"){
   document.getElementById("mqtt_connected").innerHTML = "Yes";
  } else {
    document.getElementById("mqtt_connected").innerHTML = "No";
  }

  //document.getElementById("free_heap").innerHTML = status.free_heap;
  document.getElementById("version").innerHTML = status.version;

//alert("status.mode:"+status.mode);

  if (status.mode == 'AP') {
      document.getElementById("mode").innerHTML = "Access Point (AP)";
      document.getElementById("client-view").style.display = 'none';
      document.getElementById("ap-view").style.display = '';

      var out = "";
      for (var z in status.networks) {
        if (status.rssi[z]=="undefined") status.rssi[z]="";
        out += "<tr><td><input class='networkcheckbox' name='"+status.networks[z]+"' type='checkbox'></td><td>"+status.networks[z]+"</td><td>"+status.rssi[z]+"</td></tr>"
      }
      document.getElementById("networks").innerHTML = out;
      var networkcheckboxes = document.getElementsByClassName("networkcheckbox");
      for (var i = 0; i < networkcheckboxes.length; i++) {
          networkcheckboxes[i].addEventListener('click', networkSelect, false);
      }
  } else {
      if (status.mode=="STA+AP") {
          document.getElementById("mode").innerHTML = "Client + Access Point (STA+AP)";
          document.getElementById("apoff").style.display = '';
      }
      if (status.mode=="STA") document.getElementById("mode").innerHTML = "Client (STA)";

      var out="";
      out += "<tr><td>"+status.ssid+"</td><td>"+status.srssi+"</td></tr>"
      document.getElementById("sta-ssid").innerHTML = out;
      document.getElementById("sta-ip").innerHTML = "<a href='http://"+status.ipaddress+"'>"+status.ipaddress+"</a>";
      //document.getElementById("ap-view").style.display = 'none';
      document.getElementById("ap-view").style.display = '';
      document.getElementById("client-view").style.display = '';
      ipaddress = status.ipaddress;

      var out = "";
      for (var z in status.networks) {
        if (status.rssi[z]=="undefined") status.rssi[z]="";
        out += "<tr><td><input class='networkcheckbox' name='"+status.networks[z]+"' type='checkbox'></td><td>"+status.networks[z]+"</td><td>"+status.rssi[z]+"</td></tr>"
      }
      document.getElementById("networks").innerHTML = out;
      var networkcheckboxes = document.getElementsByClassName("networkcheckbox");
      for (var i = 0; i < networkcheckboxes.length; i++) {
          networkcheckboxes[i].addEventListener('click', networkSelect, false);
      }

  }
  //updateLastValues();
};
r1.send();

setInterval(updateStatus,10000);
// -----------------------------------------------------------------------
// Periodic 10s update of last data values and packets sent
// -----------------------------------------------------------------------
function updateStatus() {
    var r2 = new XMLHttpRequest();
    r2.open("GET", "status", true);
    r2.onreadystatechange = function () {
      if (r2.readyState != 4) {
        return;
      }

      if(r2.status == 200) {
  		var status_fix = r2.responseText.replace(/( �)|( �)/gi, "");
  		var status = JSON.parse(status_fix);

        //document.getElementById("free_heap").innerHTML = status.free_heap;
        if (status.mqtt_connected == "1"){
         document.getElementById("mqtt_connected").innerHTML = "Yes";
        } else {
         document.getElementById("mqtt_connected").innerHTML = "No";
        }
        document.getElementById("save-mqtt").innerHTML = "설정 적용 (save)";

        if ((status.mode=="STA") || (status.mode=="STA+AP") || (status.mode=="AP")){
          // Update connected network RSSI
          var out="";
          out += "<tr><td>"+status.ssid+"</td><td>"+status.srssi+"</td></tr>"
          document.getElementById("sta-ssid").innerHTML = out;


          var status_fix = r2.responseText.replace(/( �)|( �)/gi, "");
      		var status = JSON.parse(status_fix);
          //console.log(str);
          var out = "";
          for (var z in status.networks) {
            if (status.rssi[z]=="undefined") status.rssi[z]="";
            out += "<tr><td><input class='networkcheckbox' name='"+status.networks[z]+"' type='checkbox'></td><td>"+status.networks[z]+"</td><td>"+status.rssi[z]+"</td></tr>"
          }
          document.getElementById("wifiscan").innerHTML = "검색 (scan)";
          document.getElementById("networks").innerHTML = out;
          var networkcheckboxes = document.getElementsByClassName("networkcheckbox");
          for (var i = 0; i < networkcheckboxes.length; i++) {
              networkcheckboxes[i].addEventListener('click', networkSelect, false);
          }

        }
      }
      //updateLastValues();
    };
    r2.send();
}
// -----------------------------------------------------------------------

function updateWiFiStatus() {
  // Update status on Wifi connection
  var r1 = new XMLHttpRequest();
  r1.open("GET", "status", true);
  r1.timeout = 2000;
  r1.onreadystatechange = function () {
    if (r1.readyState != 4) {
      return;
    }

    if(r1.status == 200) {
      var status_fix = r1.responseText.replace(/( �)|( �)/gi, "");
  		var status = JSON.parse(status_fix);
      if (status.mode=="STA+AP" || status.mode=="STA") {
        // Hide waiting message
        document.getElementById("wait-view").style.display = 'none';
        // Display mode
        if (status.mode=="STA+AP") {
            document.getElementById("mode").innerHTML = "Client + Access Point (STA+AP)";
            document.getElementById("apoff").style.display = '';
        }
        if (status.mode=="STA") document.getElementById("mode").innerHTML = "Client (STA)";
        document.getElementById("sta-ssid").innerHTML = status.ssid;
        document.getElementById("sta-ip").innerHTML = "<a href='http://"+status.ipaddress+"'>"+status.ipaddress+"</a>";

        // View display
        document.getElementById("ap-view").style.display = '';
        document.getElementById("client-view").style.display = '';
      }
    }
    lastmode = status.mode;
  };
  r1.send();
}
// -----------------------------------------------------------------------
// Event: Access Point wifiscan
// -----------------------------------------------------------------------
document.getElementById("wifiscan").addEventListener("click", function(e) {
    var r3 = new XMLHttpRequest();
    document.getElementById("wifiscan").innerHTML = "검색 시작...";

    r3.open("GET", "scan", true);
    r3.timeout = 5000;

    r3.onreadystatechange = function () {
      if (r3.readyState != 4) {
        return;
      }

      if(r3.status == 200) {
        document.getElementById("wifiscan").innerHTML = "검색중...";
      }
	  };
    r3.send();
});

// -----------------------------------------------------------------------
// Event: WiFi Connect
// -----------------------------------------------------------------------
document.getElementById("connect").addEventListener("click", function(e) {
    var passkey = document.getElementById("passkey").value;
    if (selected_network_ssid=="") {
        alert("Please select network");
    } else {
        document.getElementById("ap-view").style.display = '';
        document.getElementById("wait-view").style.display = '';

        var r = new XMLHttpRequest();
        r.open("POST", "savenetwork", false);
        r.setRequestHeader("Content-type","application/x-www-form-urlencoded");
        r.onreadystatechange = function () {
	        if (r.readyState != 4 || r.status != 200) return;
	        var str = r.responseText;
	        console.log(str);
	        document.getElementById("connect").innerHTML = "잠시후 재시작합니다...새로운 AP로 재접속하세요";

	        statusupdate = setInterval(updateWiFiStatus, 5000);
        };
        r.send("ssid="+selected_network_ssid+"&pass="+passkey);
    }
});

// -----------------------------------------------------------------------
// Event: MQTT save
// -----------------------------------------------------------------------
document.getElementById("save-mqtt").addEventListener("click", function(e) {
    var mqtt = {
      server: document.getElementById("mqtt_server").value,
      user: document.getElementById("mqtt_user").value,
      pass: document.getElementById("mqtt_pass").value
    }
    if (mqtt.server=="") {
      alert("Please enter MQTT server");
    } else {
      document.getElementById("save-mqtt").innerHTML = "저장중...";
      var r = new XMLHttpRequest();
      r.open("POST", "savemqtt", true);
      r.setRequestHeader("Content-type","application/x-www-form-urlencoded");
      r.send("&server="+mqtt.server+"&topic="+mqtt.topic+"&prefix="+mqtt.prefix+"&user="+mqtt.user+"&pass="+mqtt.pass);
      r.onreadystatechange = function () {
        console.log(mqtt);
        if (r.readyState != 4 || r.status != 200) return;
        var str = r.responseText;
  	    console.log(str);
  	    if (str!=0) document.getElementById("save-mqtt").innerHTML = "저장완료";
      };
    }
});

// -----------------------------------------------------------------------
// Event: Admin save
// -----------------------------------------------------------------------
document.getElementById("save-admin").addEventListener("click", function(e) {
    var admin = {
      user: document.getElementById("www_user").value,
      pass: document.getElementById("www_pass").value
    }
    document.getElementById("save-admin").innerHTML = "Saving...";
    var r = new XMLHttpRequest();
    r.open("POST", "saveadmin", true);
    r.setRequestHeader("Content-type","application/x-www-form-urlencoded");
    r.send("&user="+admin.user+"&pass="+admin.pass);
    r.onreadystatechange = function () {
      console.log(admin);
      if (r.readyState != 4 || r.status != 200) return;
      var str = r.responseText;
	    console.log(str);
	    if (str!=0) document.getElementById("save-admin").innerHTML = "Saved";
    };
});
// -----------------------------------------------------------------------
// Event: command
// -----------------------------------------------------------------------
document.getElementById("save-command").addEventListener("click", function(e) {
    var command = {
      exec: document.getElementById("command").value,
    }
    document.getElementById("save-command").innerHTML = "명령 실행...";
    var r = new XMLHttpRequest();
    r.open("POST", "savecommand", true);
    r.setRequestHeader("Content-type","application/x-www-form-urlencoded");
    r.send("&exec="+command.exec);
    r.onreadystatechange = function () {
      console.log(command);
      if (r.readyState != 4 || r.status != 200) return;
      var str = r.responseText;
	    console.log(str);
	    if (str!=0) document.getElementById("save-command").innerHTML = "명령(command)";
    };
});
// -----------------------------------------------------------------------
// Event: Turn off Access Point
// -----------------------------------------------------------------------
document.getElementById("apoff").addEventListener("click", function(e) {
    var r = new XMLHttpRequest();
    r.open("POST", "apoff", true);
    r.onreadystatechange = function () {
        if (r.readyState != 4 || r.status != 200) return;
        var str = r.responseText;
        console.log(str);
        document.getElementById("apoff").style.display = 'none';
        if (ipaddress!="") window.location = "http://"+ipaddress;
	  };
    r.send();
});
// -----------------------------------------------------------------------
// Event: Reset config and reboot
// -----------------------------------------------------------------------
document.getElementById("reset").addEventListener("click", function(e) {

    if (confirm("CAUTION: Do you really want to Factory Reset? All setting and config will be lost.")){
      var r = new XMLHttpRequest();
      r.open("POST", "reset", true);
      r.onreadystatechange = function () {
          if (r.readyState != 4 || r.status != 200) return;
          var str = r.responseText;
          console.log(str);
          if (str!=0) document.getElementById("reset").innerHTML = "Resetting...";
  	  };
      r.send();
    }
});
// -----------------------------------------------------------------------
// Event: Restart
// -----------------------------------------------------------------------
/*
document.getElementById("restart").addEventListener("click", function(e) {

    if (confirm("Restart emonESP? Current config will be saved, takes approximately 10s.")){
      var r = new XMLHttpRequest();
      r.open("POST", "restart", true);
      r.onreadystatechange = function () {
          if (r.readyState != 4 || r.status != 200) return;
          var str = r.responseText;
          console.log(str);
          if (str!=0) document.getElementById("reset").innerHTML = "Restarting";
  	  };
      r.send();
    }
});
*/
// -----------------------------------------------------------------------
// UI: Network select
// -----------------------------------------------------------------------
var networkSelect = function() {
    selected_network_ssid = this.getAttribute("name");

    for (var i = 0; i < networkcheckboxes.length; i++) {
        if (networkcheckboxes[i].getAttribute("name")!=selected_network_ssid)
            networkcheckboxes[i].checked = 0;
    }
};

// -----------------------------------------------------------------------
// Event:Upload Firmware
// -----------------------------------------------------------------------
document.getElementById("upload").addEventListener("click", function(e) {
  window.location.href='/upload'
});
