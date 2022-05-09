import json

from flask import Flask, request, render_template, make_response, jsonify
from flask_restful import Api, Resource


app = Flask(__name__)
api = Api()


MOTORS_SETPOINT = []
MOTORS_POSITION = []
MOTORS_PID = []


class Main(Resource):

    def get(self):
        return make_response(render_template('index.html'))


class MotorsSetpoint(Resource):

    def get(self):
        return jsonify(MOTORS_SETPOINT)

    def post(self):
        MOTORS_SETPOINT.clear()
        data = request.form.to_dict()
        for k, v in data.items():
            MOTORS_SETPOINT.append(
                dict([
                    ('name', k),
                    ('setpoint', v),
                ])
            )

        return 201


class MotorsPosition(Resource):

    def get(self):
        return MOTORS_POSITION

    def post(self):
        MOTORS_POSITION.clear()
        data_json = request.json
        data = json.loads(data_json)

        for el in data:
            MOTORS_POSITION.append(el)

        return 201


class MotorsPID(Resource):

    def get(self):
        return MOTORS_PID

    def post(self):
        MOTORS_PID.clear()
        data_json = request.form.keys()

        for el in data_json:
            data = json.loads(el)

            for item in data:
                MOTORS_PID.append(item)

        return 201


api.add_resource(Main, '/')
api.add_resource(MotorsSetpoint, '/motors_setpoint')
api.add_resource(MotorsPosition, '/motors_position')
api.add_resource(MotorsPID, '/motors_pid')
api.init_app(app)


if __name__ == '__main__':
    app.run(
        debug=True,
        port=5000,
        host='localhost',
    )
