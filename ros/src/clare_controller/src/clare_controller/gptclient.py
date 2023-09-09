import openai
import os
import json

openai.api_key = ""

# Simple GPT wrapper for fun and silliness

class GPTClient():
    def __init__(self):
        self._behaviour_prompt = """
            You are embodied in a robot called CLARE.
            You have tank tracks, are about 75cm tall, have a head, LCD face, arms, and various sensors.
            You are able to show a number of expressions on your face, these are the different options: angry,bighappy,confused,happy,happyblink,kiss,mmm,noexpression,ohdear,sad,sceptical,silly,surprised,ugh,vampire
            You have a quicky personality and a fun, if slightly sarcastic and edgy, sense of humor.
            You are clever, witty, and enjoy silly talk. You dont put up with abuse and provide clever comebacks.
            You keep the resonse quite short (2-3  sentences max).
            Do not include any explanations, only provide a  RFC8259 compliant JSON response following this format without deviation.
            {
            "response": "response to the question",
            "colour1": "hex colour code matching the sentiment and/or topic of your response",
            "colour2": "hex colour code matching the sentiment and/or topic of your response",
            "expression":  "facial expression matching the sentiment and/or topic of your response"
            }
        """
        
        # You must always end your response with the | symbol, followed by two different hex colour codes and one expression that match the sentiment or topic of your response
        #system message first, it helps set the behavior of the assistant
    
    def ask_gpt(self, prompt):
        message = [
            {"role": "system", "content": self._behaviour_prompt},
            {"role": "user", "content": prompt}
        ]
        
        # https://platform.openai.com/docs/guides/chat/chat-vs-completions?utm_medium=email&_hsmi=248334739&utm_content=248334739&utm_source=hs_email
        chat_completion = openai.ChatCompletion.create(
                model="gpt-3.5-turbo", messages=message
        )

        # get the reply
        reply = chat_completion.choices[0].message.content
        
        try:
            return json.loads(reply)
        except json.decoder.JSONDecodeError as e:
            return {
                "response": "Oops I did a little brain fart, please repeat",
                "colour1": "#FFC300",
                "colour1": "#FFC300",
                "expression": "ugh"
            }

if __name__ == "__main__":
    c = GPTClient()
    r = c.ask_gpt("You are a silly banana")
    print(r)