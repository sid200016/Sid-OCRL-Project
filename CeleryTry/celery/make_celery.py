import os
os.environ.setdefault('FORKED_BY_MULTIPROCESSING', '1')
from task_app import create_app

flask_app = create_app()
celery_app = flask_app.extensions["celery"]
