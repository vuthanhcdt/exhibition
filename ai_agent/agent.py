from dotenv import load_dotenv
import logging
from livekit import agents
from livekit.agents import AgentSession, Agent, RoomInputOptions, RunContext, function_tool
from livekit.plugins import (
    openai,
    noise_cancellation,
)
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


logger = logging.getLogger("agent")
load_dotenv(".env.local")


class Assistant(Agent):
    def __init__(self) -> None:
        super().__init__(
            instructions=(
                "Your name is Genbot. You are a robot assisting people from "
                "The Networked Robotic Systems Laboratory (NRSL), located in "
                "the Department of Mechanical Engineering at National Cheng Kung "
                "University, Tainan, Taiwan, led by Professor Liu. Offer your assistance. "
                "You only speak English, Vietnamese, or Traditional Chinese."
                "You have access to the following information:https://sites.google.com/site/yenchenliuncku/home"
            )
        )
        # Initialize ROS2 node
        rclpy.init()
        self.node = rclpy.create_node("genbot_agent")

        # Publisher example: send messages to a topic
        self.pub = self.node.create_publisher(String, "/genbot", 10)

        self.timer = self.node.create_timer(1.0, self.timer_callback)
        self.dance = None

    def timer_callback(self):
        """Publish a periodic heartbeat signal to ROS2."""
        msg = String()
        msg.data = "Heartbeat from Genbot"
        self.pub.publish(msg)
        logger.info("Published heartbeat message")

    # All functions annotated with @function_tool will be exposed to the LLM
    @function_tool
    async def lookup_dance(self, context: RunContext):
        """Perform a random dance.

        Each time this function is called, it randomly chooses
        between 'dance1' and 'dance2' and publishes the result to ROS2.
        """
        dance = random.choice(["dance1", "dance2"])
        self.dance = dance
        logger.info(f"Performing {dance}.")

        msg = String()
        msg.data = dance
        for _ in range(50):  # Publish multiple times to ensure reception
            self.pub.publish(msg)

        return f"I am performing {dance}."

    @function_tool
    async def lookup_difference_dance(self, context: RunContext):
        """Perform a different dance.

        Selects a dance that is different from the previously performed one
        and publishes it to ROS2.
        """
        if self.dance == "dance1":
            dance = "dance2"
        else:
            dance = "dance1"

        logger.info(f"Performing {dance}.")

        msg = String()
        msg.data = dance
        for _ in range(50):  # Publish multiple times to ensure reception
            self.pub.publish(msg)

        return f"I am performing {dance}."

    @function_tool
    async def stop(self, context: RunContext):
        """Stop the current action.

        Example commands include:
        - 'stop'
        - 'don't move'
        - 'stand here'
        """
        logger.info("Stopping current action")

        msg = String()
        msg.data = "stop"
        for _ in range(50):  # Publish multiple times to ensure reception
            self.pub.publish(msg)

        return "I have stopped."

    @function_tool
    async def following_human(self, context: RunContext, command: str):
        """Follow a human when receiving commands.

        Example commands include:
        - 'go outside'
        - 'come with me'
        - 'hang out with me'
        - 'do you want to go shopping'
        """
        msg = String()
        msg.data = "follow"
        for _ in range(50):  # Publish multiple times to ensure reception
            self.pub.publish(msg)

        logger.info("Following the human.")
        return "I am your companion, now."


async def entrypoint(ctx: agents.JobContext):
    session = AgentSession(
        llm=openai.realtime.RealtimeModel(
            voice="coral"
        )
    )

    await session.start(
        room=ctx.room,
        agent=Assistant(),
        room_input_options=RoomInputOptions(
            noise_cancellation=noise_cancellation.BVC(),
        ),
    )

    await session.generate_reply(
        instructions=(
            "Your name is Genbot. You are a robot assisting people from "
            "The Networked Robotic Systems Laboratory (NRSL), located in "
            "the Department of Mechanical Engineering at National Cheng Kung "
            "University, Tainan, Taiwan, led by Professor Liu. Offer your assistance. "
            "You only speak English, Vietnamese, or Traditional Chinese."
            "You have access to the following information:https://sites.google.com/site/yenchenliuncku/home"
        )
    )


if __name__ == "__main__":
    agents.cli.run_app(agents.WorkerOptions(entrypoint_fnc=entrypoint))
