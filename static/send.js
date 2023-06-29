function send_gantry_data(){

    let mouseX = event.clientX;
    let mouseY = event.clientY;
    var rect = event.target.getBoundingClientRect();

    var x = mouseX - rect.left; //x position within the element.
    var y = mouseY - rect.top;  //y position within the element.


    let value = x.toString() + ',' + y.toString();

    $.ajax({
            url: '/participant/dual-grasper',
            type: 'POST',
            data: { 'gantry': value },
            success: function(response) {
                // document.getElementById('output').innerHTML = response;
                console.log(x + ', ' + y);
                return;
            },
            error: function(error) {
                console.log(error);
            }
    });
}

// mouse position is off
function send_grasper_data_left() {
    let mouseY = event.clientY;
    let rect = event.target.getBoundingClientRect();

    let y = rect.bottom - mouseY

    let value = (100*(y/rect.height)).toString(); // need to convert to percent

    $.ajax({
            url: '/participant/dual-grasper',
            type: 'POST',
            data: { 'grasper_l': value },
            success: function(response) {
                // document.getElementById('output').innerHTML = response;
                console.log(value);
                return;
            },
            error: function(error) {
                console.log(error);
            }
    });
}

function send_grasper_data_right() {
    let mouseY = event.clientY;
    let rect = event.target.getBoundingClientRect();

    let y = rect.bottom - mouseY

    let value = (100*(y/rect.height)).toString(); // convert to percent

    $.ajax({
            url: '/participant/dual-grasper',
            type: 'POST',
            data: { 'grasper_r': value },
            success: function(response) {
                // document.getElementById('output').innerHTML = response;
                console.log(value);
                return;
            },
            error: function(error) {
                console.log(error);
            }
    });
}

let graspser_on = -1;

function activate_grasper(){
    graspser_on *= -1;
    let value = graspser_on.toString();

    $.ajax({
        url: '/participant/dual-grasper',
        type: 'POST',
        data: { 'grasper_on': value },
        success: function(response) {
            // document.getElementById('output').innerHTML = response;
            console.log(value);
            return;
        },
        error: function(error) {
            console.log(error);
        }
    });
}