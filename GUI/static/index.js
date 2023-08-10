/**** globals ****/

const socket = io({autoConnect: false});
let users = {};
let attempts = [];
let NUM_ATTEMPTS = 12;

const attempt_colors = {
    'success': '#159F20',
    'fail': '#E24D4D',
    'inprogress': '#498DF2',
    'broken': '#ECE769'
}

/**** setup -> landing ****/
document.getElementById("proc-join").addEventListener("click", function() {
    socket.connect();

    socket.on("connect", function() {
        socket.emit("proc-join");
    })
    
    users['proctor'] = socket.id;
    console.log('proctor joined!');

    document.getElementById("setup").style.display = "none";
    document.getElementById("landing-proc").style.display = "block";
    document.getElementById("header-bar-proc").style.display = "block";

})

document.getElementById("part-join").addEventListener("click", function() {
    socket.connect();

    socket.on("connect", function() {
        socket.emit("part-join");
    })

    users['participant'] = socket.id;
    console.log('participant joined!')

    document.getElementById("setup").style.display = "none";
    document.getElementById("landing-part").style.display = "block";
    document.getElementById("header-bar-part").style.display = "block";
})

/**** landing ****/
let soft_first = true;
let soft_select = document.getElementById("soft-selector");
let rigid_select = document.getElementById("rigid-selector");

soft_select.addEventListener("click", function() {
    soft_first = true;
    toggle_grasper();
})

rigid_select.addEventListener("click", function() {
    soft_first = false;
    toggle_grasper();
})

function toggle_grasper(){
    if (soft_first) {
        soft_select.style.background = "#70CA68";
        rigid_select.style.background = "#775489";
    } else {
        soft_select.style.background = "#775489";
        rigid_select.style.background = "#70CA68";
    }
}


/**** landing -> action ****/
document.getElementById("done").addEventListener("click", function() {

    let id_code = document.getElementById('id-code').value
    let age = document.getElementById('age').value
    let gender = document.getElementById('gender').value

    let item_types = [];
    for (let i = 0; i < 9; i++){
        let item_type = document.getElementsByName('type' + i.toString())[0].value;
        item_types.push(item_type);
    }

    let user_data = {id_code: id_code, age: age, gender: gender, item_types: item_types};
    
    let test = 'rigid';
    if (soft_first) {
        test = 'soft';
    }

    let trial_data = {test: test, num_attempts: NUM_ATTEMPTS};

    document.getElementById("landing-proc").style.display = "none";
    document.getElementById(test + "-grasper-part").style.display = "none";
    document.getElementById(test + "-grasper-proc").style.display = "block";

    document.getElementById("range").disabled = true; 

    socket.emit("info-done", user_data, trial_data);

})

socket.on("goto-soft", function(){
    document.getElementById("landing-part").style.display = "none";
    document.getElementById("waiting-part").style.display = "none";
    document.getElementById("soft-grasper-part").style.display = "block";
})

socket.on("goto-rigid", function(){
    document.getElementById("landing-part").style.display = "none";
    document.getElementById("waiting-part").style.display = "none";
    document.getElementById("rigid-grasper-part").style.display = "block";
})


let gantry = document.getElementById("slider-2d")
function emit_gantry_data() {
    let gx = gantry.lastChild.getAttribute("cx")
    let gy = gantry.lastChild.getAttribute("cy")
    socket.emit("gantry-move", {x: gx, y: gy});
}
gantry.addEventListener("mousedown", function() {
    emit_gantry_data();
    gantry.addEventListener("mousemove", emit_gantry_data)
}) 
gantry.addEventListener("mouseup", function() {
    emit_gantry_data();
    gantry.removeEventListener("mousemove", emit_gantry_data)
})
gantry.addEventListener("mouseout", function() {
    gantry.removeEventListener("mousemove", emit_gantry_data)
})

let slider_damping = document.getElementById("slider-damping")
let slider_stiffness = document.getElementById("slider-stiffness")
let slider_power = document.getElementById("slider-power")

slider_damping.addEventListener("input", function() {
    console.log(slider_damping.value);
    document.getElementById("damping-soft-value").innerHTML = Math.round(slider_damping.value).toString() + '%'
    socket.emit("damping-change", slider_damping.value);
    console.log(slider_damping.value)
})
slider_stiffness.addEventListener("input", function() {
    document.getElementById("stiffness-soft-value").innerHTML = Math.round(slider_stiffness.value).toString() + '%'
    socket.emit("stiffness-change", slider_stiffness.value);
    console.log(slider_stiffness.value)
})
slider_power.addEventListener("input", function() {
    document.getElementById("power-soft-value").innerHTML = Math.round(slider_power.value).toString() + '%'
    socket.emit("power-change", slider_power.value);
    console.log(slider_power.value)
}) 

// *************************************

function item_event_selection(test){
    for (let row = 0; row < 3; row++){
        for (let col = 0; col < 3; col++){
            (function () {
                let n = row * 3 + col;
                let curr_item = document.getElementById("item" + n.toString() + '-proc' + '-' + test);
                let curr_menu = document.getElementById("item-menu" + n.toString() + '-' + test);
                curr_item.addEventListener("mouseover", function() {
                    curr_menu.style.display = "flex";
                })

                let success = document.getElementById("success" + n.toString() + '-' + test);
                let fail = document.getElementById("fail" + n.toString() + '-' + test);
                let inprogress = document.getElementById("inprogress" + n.toString() + '-' + test);
                let broken = document.getElementById("broken" + n.toString() + '-' + test);

                success.addEventListener("click", function() {
                    socket.emit("item-event", {'n': n, 'typ': 'success', att: attempts, test: test});
                })
                fail.addEventListener("click", function() {
                    socket.emit("item-event", {'n': n, 'typ': 'fail', att: attempts, test: test});
                })
                inprogress.addEventListener("click", function() {
                    socket.emit("item-event", {'n': n, 'typ': 'inprogress', att: attempts, test: test});
                })
                broken.addEventListener("click", function() {
                    socket.emit("item-event", {'n': n, 'typ': 'broken', att: attempts, test: test});
                })

                curr_menu.addEventListener("mouseleave", function() {
                    curr_menu.style.display = "none";
                })
            }());

        }
    }
}

item_event_selection('soft');
item_event_selection('rigid');

function choose_random_item(target, test){
    let attempted_items = [];
    for (let i = 0; i < attempts.length; i++) {
        attempted_items.push(attempts[i]['n']);
    }

    let n = Math.floor(9 * Math.random());
    while (attempted_items.includes(n) || attempted_items.length == 9) {
        n = Math.floor(9 * Math.random());
    }
    let item = document.getElementById('item' + n.toString() + '-' + target + '-' + test);
    item.src = "static/public/attempt-target.svg";

    return n;
}

socket.on("new-target-item-soft", function() {
    let n = choose_random_item('proc', 'soft');
    socket.emit("send-target-item", n)
})

socket.on("update-target-item-soft", function(n) {
    let item = document.getElementById('item' + n.toString() + '-part-soft');
    item.src = "static/public/attempt-target.svg";
})

socket.on("new-target-item-rigid", function() {
    let n = choose_random_item('proc', 'rigid');
    socket.emit("send-target-item", n)
})

socket.on("update-target-item-rigid", function(n) {
    let item = document.getElementById('item' + n.toString() + '-part-rigid');
    item.src = "static/public/attempt-target.svg";
})

function update_events(data, target, test) {

    let att = data['att'];
    let typ = data['typ'];
    let n = data['n'];

    document.getElementById("item" + n.toString() + '-' + target + '-' + test).src = "static/public/attempt-" + typ + ".svg";

    if (att.length > 0 && att[att.length-1]['typ'] == 'inprogress'){
        attempts[attempts.length-1] = {n: n, typ: typ};
    } else {
        attempts.push({n: n, typ: typ})
    }

    let attempts_made = attempts.length;
    let attempt_marker = document.getElementById('attempt'+(attempts_made-1).toString()+ '-' + target + '-' + test);
    attempt_marker.style.background = attempt_colors[typ];

    attempts_remaining = document.getElementById("attempts-remaining-" + target + '-' + test);
    attempts_remaining.innerHTML = (NUM_ATTEMPTS - attempts_made).toString() + ' attempts remaining';

    let all_success = false;
    if (attempts.length >= 9) {
        all_success = true;
        for (let i = 0; i < attempts.length; i++){
            if (attempts[i]['typ'] != 'success'){
                all_success = false;
            }
        }
    }
    if (NUM_ATTEMPTS == attempts_made || all_success) {
        if (test == 'soft'){
            document.getElementById("soft-grasper-" + target).style.display = "none";
            document.getElementById("waiting-" + target).style.display = "block";
            let soft_attempts = attempts;
            attempts = [];
        } else if (test == 'rigid') {
            document.getElementById("rigid-grasper-" + target).style.display = "none";
            document.getElementById("results-" + target).style.display = "block";
            let rigid_attempts = attempts;
            attempts = [];
        }
    }

}

function update_event_proc_soft(data){
    update_events(data, 'proc', 'soft');
}

function update_event_part_soft(data){
    update_events(data, 'part', 'soft');
}

function update_event_proc_rigid(data){
    update_events(data, 'proc', 'rigid');
}

function update_event_part_rigid(data){
    update_events(data, 'part', 'rigid');
}

socket.on("update-events-proctor-soft", update_event_proc_soft);
socket.on("update-events-participant-soft", update_event_part_soft);
socket.on("update-events-proctor-rigid", update_event_proc_rigid);
socket.on("update-events-participant-rigid", update_event_part_rigid);

/**** waiting -> action ****/
document.getElementById("waiting-proc").addEventListener("click", function() {

    if (soft_first == true) test = 'rigid';
    else test = 'soft';

    document.getElementById("waiting-proc").style.display = "none";
    document.getElementById(test + "-grasper-proc").style.display = "block";

    document.getElementById("range").disabled = true; 

    socket.emit("goto-action", test);

})

socket.on("set-contact-force-soft", function(val) {
    document.getElementById("contact-force-soft").value = Number(val);
    document.getElementById("contact-force-soft-value").innerHTML = Math.round(Number(val)).toString() + '%';
})

socket.on("set-contact-force-rigid", function(val) {
    document.getElementById("contact-force-rigid").value = Number(val);
    document.getElementById("contact-force-rigid-value").innerHTML = Math.round(Number(val));
})

socket.on("set-gantry-marker", function(data){
    let marker = document.getElementById("gantry-marker")
    let r = marker.getBoundingClientRect().width;
    marker.style.left = -mm_to_mouse(data['x']) - r/2;
    marker.style.top = mm_to_mouse(data['y']) - r/2;

})
