import time

from celery import shared_task
from celery import Task


@shared_task(ignore_result=False)
def add(a: int, b: int) -> int:
    return a + b


@shared_task()
def block() -> None:
    time.sleep(5)

@shared_task()
def init() -> None:
    print("I'm drowsy, can I sleep some more?...")

    print("Maybe just a little more...")
    time.sleep(300)
    print('Done sleeping, I am awake')

@shared_task(bind=True, ignore_result=False)
def process(self: Task, total: int) -> object:
    for i in range(total):
        self.update_state(state="PROGRESS", meta={"current": i + 1, "total": total})
        time.sleep(1)

    return {"current": total, "total": total}
