from flask import Flask, render_template, request, url_for, flash, redirect
from datetime import datetime
from gantry import *
import csv

app = Flask(__name__)
app.config['SECRET_KEY'] = 'ceff418fb561ebf2572221b1f28789a36e4e30f7da4df0a8'

# Gantry / Grasper setup
Gantry = Gantry()
Grasper = Grasper()

# date / time
MONTHS = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']

now = datetime.now()
date = f'{MONTHS[now.month - 1]} - {now.day} - {now.year}'

hour = now.hour if now.hour <= 12 else now.hour - 12
minute = now.minute if now.minute >= 10 else '0'+now.minute
ampm = 'A.M' if now.hour <= 12 else 'P.M'
time = f'{hour}:{now.minute} {ampm}'

def log_participant_data(id_code, age, gender, types):
    with open(f'{id_code}.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        
        writer.writerow(["id code", "age", "gender"])
        writer.writerow([id_code, age, gender])
        writer.writerow([])
        writer.writerow([f'type{i}' for i in range(len(types))])
        writer.writerow(types)

def log_trial_data(timestamp, gantry_x, gantry_y, objects):
    pass

def mouse_to_mm(mouse_position):
    # fix this
    return mouse_position

def mm_to_mouse(mm):
    # fix this
    return mm

@app.route("/", methods=('GET', 'POST'))
@app.route("/home", methods=('GET', 'POST'))
def index():
    if request.method == 'POST':
        types = []
        for i in range(9):
            types[i] = request.form[f'type{i}']
        
        id_code = request.form['id-code']
        age = request.form['age']
        gender = request.form['gender']
        
        if not id_code:
            flash('Id Code is required!')
        elif not age:
            flash('Age is required!')
        elif not gender:
            flash('Gender is required!')
        else:
            log_participant_data(id_code, age, gender, types)
            return redirect(url_for('soft_grasper'))

    return render_template("index.html", date=date, time=time)

@app.route("/participant/dual-grasper", methods=('GET', 'POST'))
def dual_grasper_participant():
    # read user inputs from GUI
    data_string = request.form.get('data')
    # data_string: 'mouse_x,mouse_y,power_l,power_r'
    if data_string:
        data = data_string.split(',')

        data[0] = mouse_to_mm(data[0])
        data[1] = mouse_to_mm(data[1])

        # print("mouseX: " + data[0] + ', mouseY: ' + data[1])

        # g_x, g_y = Gantry.GetPosition()
        # p_l, p_r = Grasper.GetPower()
        # real_data = [g_x, g_y, p_l, p_r]

        # for i in range(len(data)):
        #     if data[i] == 'null':
        #         # if user provides no data send back the measued data
        #         data[i] = real_data[i]

        Gantry.SetPosition(data[0], data[1])
        # Grasper.SetPower(data[2], data[3])

    return render_template( "dual-grasper-PARTICIPANT.jinja", 
                            # mouseX=mm_to_mouse(data[0]), mouseY=mm_to_mouse(data[1]),
                            # power_l=data[2], power_r=data[3],
                            date=date, time=time
                            )