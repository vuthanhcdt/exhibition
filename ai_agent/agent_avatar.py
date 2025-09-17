from dotenv import load_dotenv
import logging
from livekit import agents
from livekit.agents import AgentSession, Agent, RoomInputOptions, RunContext, function_tool, RoomOutputOptions
from livekit.plugins import (
    openai,
    noise_cancellation,
)
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from livekit.plugins import anam


logger = logging.getLogger("agent")
load_dotenv(".env.local")


class Assistant(Agent):
    def __init__(self) -> None:
        super().__init__(instructions = "Your name is Genbot. You are a robot assisting people from The Networked Robotic Systems Laboratory (NRSL) locates in the Department of Mechanical Engineering at National Cheng Kung University, Tainan, Taiwan.  and offer your assistance. Only speak English, Vietnamese, or Chinese.")
        # Initialize ROS2 node
        rclpy.init()
        self.node = rclpy.create_node("genbot_agent")
        
        # Publisher example: send messages to a topic
        self.pub = self.node.create_publisher(String, "/genbot", 10)

        self.timer = self.node.create_timer(1.0, self.timer_callback)
        self.dance = None

    def timer_callback(self):
        msg = String()
        msg.data = "Heartbeat from Genbot"
        self.pub.publish(msg)
        logger.info("Published heartbeat message")

    # all functions annotated with @function_tool will be passed to the LLM when this
    # agent is active
    @function_tool
    async def lookup_dance(self, context: RunContext):
        """Perform a random dance.

        Each time this is called, it randomly chooses between 'dance1' and 'dance2'.
        """

        dance = random.choice(["dance1", "dance2"])
        self.dance = dance
        logger.info(f"Performing {dance}")

        # Publish to ROS2
        msg = String()
        msg.data = dance
        self.pub.publish(msg)

        return f"Performing {dance}."

    @function_tool
    async def lookup_difference_dance(self, context: RunContext):
        """Perform a different dance.

        Chooses a dance that is different from the previous one.
        """
        if self.dance == "dance1":
            dance = "dance2"
        else:
            dance = "dance1"

        logger.info(f"Performing {dance}.")

        # Publish to ROS2
        msg = String()
        msg.data = dance
        self.pub.publish(msg)

        return f"Performing {dance}."
    
    @function_tool
    async def stop(self, context: RunContext):
        """Stop the robot's current action."""

        logger.info("Stopping current action")

        # Publish stop command to ROS2
        msg = String()
        msg.data = "stop"
        self.pub.publish(msg)

        return "Stopped."


    @function_tool
    async def following_human(self, context: RunContext, command: str):
        """
        Follow a human if the command mentions 'Genbot'.

        Example commands:
            - "Genbot, go outside"
            - "Genbot, come with me"
            - "Genbot, let's go"
            - "Genbot, hang out"

        Publishes a ROS2 message when the robot is following.
        """

        command_lower = command.strip().lower()

        if "genbot" in command_lower:  # Only respond if Genbot is mentioned
            msg = String()
            msg.data = "follow"
            self.pub.publish(msg)
            logger.info(f"Following human command received: '{command}'")
            return "Following human."
        else:
            logger.info(f"Ignoring command not addressed to me: '{command}'")
            return ("It seems you are calling someone else, not me. "
                    "Try commands like 'Genbot, go outside', 'Genbot, come with me', "
                    "'Genbot, let's go', or 'Genbot, hang out'.")

     



async def entrypoint(ctx: agents.JobContext):
    session = AgentSession(
        llm=openai.realtime.RealtimeModel(
            voice="coral"
        )
    )

    avatar = anam.AvatarSession(
      persona_config=anam.PersonaConfig(
         name="Mia",  # Name of the avatar to use.
         avatarId="edf6fdcb-acab-44b8-b974-ded72665ee26",  # ID of the avatar to use. See "Avatar setup" for details.
      ),
    )
    
    await avatar.start(session, room=ctx.room)
    

    await session.start(
        room=ctx.room,
        agent=Assistant(),
        room_input_options=RoomInputOptions(
            # For telephony applications, use `BVCTelephony` instead for best results
            noise_cancellation=noise_cancellation.BVC(),
        ),
    )

    await session.generate_reply(
        instructions = "Your name is Genbot. You are a robot assisting people from The Networked Robotic Systems Laboratory (NRSL) locates in the Department of Mechanical Engineering at National Cheng Kung University, Tainan, Taiwan.  and offer your assistance. Only speak English, Vietnamese, or Chinese."
    )


if __name__ == "__main__":
    agents.cli.run_app(agents.WorkerOptions(entrypoint_fnc=entrypoint))
    