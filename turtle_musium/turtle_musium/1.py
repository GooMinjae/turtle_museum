from graphviz import Digraph

# 플로우차트 생성
dot = Digraph(comment="Robot9ToMain Flowchart", format="png")
dot.attr(rankdir="TB", size="10")

# 시작 / 초기화
dot.node("START", "Start (/robot9/person_exit)", shape="ellipse", style="filled", fillcolor="lightgrey")

# 초기 상태
dot.node("INIT", "Undock + setInitialPose", shape="box")
dot.node("IDLE", "Idle / 대기", shape="box")

# 순찰
dot.node("PATROL", "순찰 경로 실행\n(entrance→p1→p2→p3→exit)", shape="box")
dot.node("CHECK", "회화 체크\n(/paint_check 발행)", shape="parallelogram")

# 인터럽트
dot.node("INTERRUPT", "Gift Interrupt\n(/robot9/gift True)", shape="diamond", color="red")
dot.node("TO_GIFT", "CancelTask → gift_shop 이동", shape="box", style="filled", fillcolor="mistyrose")
dot.node("PUB_GIFT", "/gift_start True 발행", shape="parallelogram", style="filled", fillcolor="pink")

# Gift Stay & Guide End
dot.node("GIFT_STAY", "gift_stay 포즈 이동\n(/robot9/gift_stay)", shape="box", style="filled", fillcolor="lightyellow")
dot.node("GUIDE_END", "가이드 종료 감지\n(/robot9/guide_done)", shape="diamond")
dot.node("PUB_PERSON", "/robot9/person True 발행", shape="parallelogram", style="filled", fillcolor="lightblue")

# 종료
dot.node("END", "End / 대기 상태", shape="ellipse", style="filled", fillcolor="lightgrey")

# 흐름 연결
dot.edges([("START", "INIT"), ("INIT", "IDLE"), ("IDLE", "PATROL")])

# 순찰 시나리오
dot.edge("PATROL", "CHECK", label="painting_i 위치 도달")
dot.edge("CHECK", "PATROL", label="다음 경로 진행")
dot.edge("PATROL", "END", label="모든 경로 완료")

# 인터럽트 분기
dot.edge("PATROL", "INTERRUPT", label="/gift True")
dot.edge("INTERRUPT", "TO_GIFT", label="Yes")
dot.edge("TO_GIFT", "PUB_GIFT")
dot.edge("PUB_GIFT", "GIFT_STAY", label="/gift_stay True")

# Guide 종료
dot.edge("GIFT_STAY", "GUIDE_END")
dot.edge("GUIDE_END", "PUB_PERSON", label="Yes")
dot.edge("PUB_PERSON", "END")

flowchart_path = "/mnt/data/robot9_flowchart"
dot.render(flowchart_path, format="png", cleanup=False)

flowchart_path + ".png"
