function onCreateMenu(){
  var searchParams = new URLSearchParams(window.location.search);
  var str_url_path = window.location.pathname;
  var obj_div_elements = document.getElementById("navbar");
  
  // Index page
  var obj_a = document.createElement("a");
  obj_a.setAttribute("href", "/");
  obj_a.innerHTML = "Home";
  if (str_url_path=="/"){
    obj_a.setAttribute("class", "active");
  }
  obj_div_elements.appendChild(obj_a);
  
  // graphs page
  var obj_a = document.createElement("a");
  obj_a.setAttribute("href", "graphs");
  obj_a.innerHTML = "Graphs";
  if (str_url_path=="graphs"){
    obj_a.setAttribute("class", "active");
  }
  obj_div_elements.appendChild(obj_a);
  
  var xhr = new XMLHttpRequest();
  xhr.open("GET","params.json");
  
  // settings pages
  xhr.onload= function() {
    var str_buf = xhr.responseText;
    const obj_json_req = JSON.parse(str_buf);
    
    for(var str_key in obj_json_req){
      var obj_a = document.createElement("a");
      obj_a.setAttribute("href", "settings?section=" + str_key);
      obj_a.innerHTML = str_key;
      if (searchParams.get('section') == str_key){
        obj_a.setAttribute("class", "active");
      }
      obj_div_elements.appendChild(obj_a);
    }
  }
  xhr.send();
  
  // ota page
  var obj_a = document.createElement("a");
  obj_a.setAttribute("href", "ota");
  obj_a.innerHTML = "OTA";
  if (str_url_path=="ota"){
    obj_a.setAttribute("class", "active");
  }
  obj_div_elements.appendChild(obj_a);
  
  var obj_a = document.createElement("a");
  obj_a.setAttribute("href", "about");
  obj_a.innerHTML = "About";
  if (str_url_path=="about"){
    obj_a.setAttribute("class", "active");
  } else {
    obj_a.setAttribute("class", "right");
  }
  obj_div_elements.appendChild(obj_a);
}