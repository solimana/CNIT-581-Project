function savepos_robposB(~, message)
    global robposB
    robposB = double(message.Data);
end