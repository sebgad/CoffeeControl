function onCreateMenu(){
  let searchParams = new URLSearchParams(window.location);
  var str_url_path = window.location.pathname;
  var obj_div_elements = document.getElementById("settings");

  alert(str_url_path);
  obj_a = document.createElement("a");
  obj_a.setAttribute("href", "/");
  if (str_url_path=="/"){
    obj_a.setAttribute("class", "active");
  }

  obj_div_elements.appendChild(obj_a);


  var xhr = new XMLHttpRequest();
  xhr.open("GET","params.json");

  xhr.onload= function() {
    var str_buf = xhr.responseText;
    const obj_json_req = JSON.parse(str_buf);

    for(var str_key in obj_json_req){
      obj_a = document.createElement("a");
      obj_a.setAttribute("href", "settings/" + str_key);
      obj_a.innerHTML = "settings_" + str_key;
      obj_div_elements.appendChild(obj_a);
    }
  }

  xhr.send();
}